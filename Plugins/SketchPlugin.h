/*

Copyright (c) 2013, Project OSRM, Dennis Luxen, others
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef SKETCH_PLUGIN_H
#define SKETCH_PLUGIN_H

#include "BasePlugin.h"

#include "../Algorithms/ObjectToBase64.h"
#include "../Algorithms/SimplePathSchematization.h"

#include "../DataStructures/QueryEdge.h"
#include "../DataStructures/SearchEngine.h"
#include "../Descriptors/BaseDescriptor.h"
#include "../Descriptors/GPXDescriptor.h"
#include "../Descriptors/JSONDescriptor.h"
#include "../Util/simple_logger.hpp"
#include "../Util/StringUtil.h"
#include "../Util/TimingUtil.h"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

template <class DataFacadeT> class SketchPlugin : public BasePlugin
{
  private:
    std::shared_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;

    void computeStreetNameIntervals(DataFacadeT* facade, const std::vector<SegmentInformation>& info, JSON::Array& intervals) const
    {
        std::vector<unsigned> name_ids;
        for (unsigned i = 0; i < info.size(); i++)
        {
            if (!info[i].necessary)
            {
                continue;
            }

            name_ids.push_back(info[i].name_id);
        }

        unsigned last_start = 0;
        for (unsigned i = 0; i < name_ids.size()-1; i++)
        {
            if (name_ids[i] != name_ids[i+1])
            {
                JSON::Array interval;
                interval.values.push_back(last_start);
                interval.values.push_back(i);
                interval.values.push_back(facade->GetEscapedNameForNameID(name_ids[i]));
                intervals.values.push_back(interval);
                last_start = i;
            }
        }

        JSON::Array interval;
        interval.values.push_back(last_start);
        interval.values.push_back(name_ids.size()-1);
        interval.values.push_back(facade->GetEscapedNameForNameID(name_ids[name_ids.size()-1]));
        intervals.values.push_back(interval);
    }

    using SegmentIt = std::vector<SegmentInformation>::iterator;
    using SegmentInterval = std::pair<SegmentIt, SegmentIt>;
    SegmentInterval computeLowDetailIntervals(SegmentIt begin, SegmentIt end)
    {
        const double marker_distance = 10000;
        auto begin_low_detail_it = end;
        double total_length = 0;
        for (auto it = begin; it != std::prev(end); ++it)
        {
            total_length += FixedPointCoordinate::ApproximateEuclideanDistance(
                                it->location, std::next(it)->location);
            if (total_length > marker_distance)
            {
                begin_low_detail_it = it;
                break;
            }
        }
        auto end_low_detail_it = begin_low_detail_it;
        total_length = 0;
        if (begin_low_detail_it != end)
        {
            for (auto it = std::prev(end); it != begin_low_detail_it; --it)
            {
                total_length += FixedPointCoordinate::ApproximateEuclideanDistance(
                                    it->location, std::prev(it)->location);
                if (total_length > marker_distance)
                {
                    end_low_detail_it = it;
                    break;
                }
            }
        }

        return std::make_pair(begin_low_detail_it, end_low_detail_it);
    }

  public:
    explicit SketchPlugin(DataFacadeT *facade) : sps(2, 0.1), descriptor_string("sketch"), facade(facade)
    {
        search_engine_ptr = std::make_shared<SearchEngine<DataFacadeT>>(facade);
    }

    virtual ~SketchPlugin() {}

    const std::string GetDescriptor() const { return descriptor_string; }

    void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply)
    {
        // check number of parameters
        if (2 > route_parameters.coordinates.size() ||
            std::any_of(begin(route_parameters.coordinates),
                        end(route_parameters.coordinates),
                        [&](FixedPointCoordinate coordinate)
                        { return !coordinate.isValid(); }))
        {
            reply = http::Reply::StockReply(http::Reply::badRequest);
            return;
        }

        RawRouteData raw_route;
        raw_route.check_sum = facade->GetCheckSum();
        for (const FixedPointCoordinate &coordinate : route_parameters.coordinates)
        {
            raw_route.raw_via_node_coordinates.emplace_back(coordinate);
        }

        std::vector<PhantomNode> phantom_node_vector(raw_route.raw_via_node_coordinates.size());
        const bool checksum_OK = (route_parameters.check_sum == raw_route.check_sum);

        for (unsigned i = 0; i < raw_route.raw_via_node_coordinates.size(); ++i)
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i], phantom_node_vector[i]);
                if (phantom_node_vector[i].isValid(facade->GetNumberOfNodes()))
                {
                    continue;
                }
            }
            facade->FindPhantomNodeForCoordinate(raw_route.raw_via_node_coordinates[i],
                                                 phantom_node_vector[i],
                                                 route_parameters.zoom_level);
        }

        PhantomNodes current_phantom_node_pair;
        for (unsigned i = 0; i < phantom_node_vector.size() - 1; ++i)
        {
            current_phantom_node_pair.source_phantom = phantom_node_vector[i];
            current_phantom_node_pair.target_phantom = phantom_node_vector[i + 1];
            raw_route.segment_end_coordinates.emplace_back(current_phantom_node_pair);
        }

        const bool is_alternate_requested = route_parameters.alternate_route;
        const bool is_only_one_segment = (1 == raw_route.segment_end_coordinates.size());
        if (is_alternate_requested && is_only_one_segment)
        {
            search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
                                                raw_route);
        }
        else
        {
            search_engine_ptr->shortest_path(raw_route.segment_end_coordinates, route_parameters.uturns, raw_route);
        }


        if (INVALID_EDGE_WEIGHT == raw_route.shortest_path_length)
        {
            SimpleLogger().Write(logDEBUG) << "Error occurred, single path not found";
        }

        reply.status = http::Reply::ok;

        // construct Segments without real turn instructions
        std::vector<SegmentInformation> route_info;
        std::vector<unsigned> via_indices;
        for (unsigned i = 0; i < raw_route.unpacked_path_segments.size(); ++i)
        {
            const PhantomNodes& phantom_nodes = raw_route.segment_end_coordinates[i];
            via_indices.push_back(route_info.size());
            route_info.emplace_back(phantom_nodes.source_phantom.location,
                                    phantom_nodes.source_phantom.name_id,
                                    0, 0.f, TurnInstruction::NoTurn, true, true, TRAVEL_MODE_DEFAULT);
            for (const PathData &path_data : raw_route.unpacked_path_segments[i])
            {
                route_info.emplace_back(
                    facade->GetCoordinateOfNode(path_data.node),
                    path_data.name_id, path_data.segment_duration, 0.f,
                    path_data.turn_instruction,
                    path_data.turn_instruction != TurnInstruction::NoTurn,
                    false,
                    path_data.travel_mode);
            }
            via_indices.push_back(route_info.size());
            route_info.emplace_back(phantom_nodes.target_phantom.location,
                                    phantom_nodes.target_phantom.name_id,
                                    0, 0.f, TurnInstruction::NoTurn, true, true, TRAVEL_MODE_DEFAULT);
        }

        if (route_info.size() == 0)
        {
            reply = http::Reply::StockReply(http::Reply::badRequest);
            return;
        }

        BOOST_ASSERT(route_info.size() >= 2);

        DouglasPeucker dp;
        for (unsigned i = 0; i < via_indices.size() - 1; i++)
        {
            auto begin = route_info.begin() + via_indices[i];
            auto end   = route_info.begin() + via_indices[i+1];
            auto interval = computeLowDetailIntervals(begin, end);
            dp.Run(begin, interval.first, 12);
            dp.Run(interval.first, interval.second, 5);
            dp.Run(interval.second, end, 12);
        }


        std::vector<SegmentInformation> schematized_info;
        sps.schematize(route_info, schematized_info);

        PolylineCompressor compressor;
        JSON::String schematized_polyline = compressor.printEncodedString(schematized_info);
        JSON::String route_polyline = compressor.printEncodedString(route_info);

        JSON::Object json_object;
        json_object.values["route_geometry"] = route_polyline;
        json_object.values["schematized_geometry"] = schematized_polyline;

        JSON::Array street_intervals;
        JSON::Array schematized_street_intervals;
        computeStreetNameIntervals(facade, route_info, street_intervals);
        computeStreetNameIntervals(facade, schematized_info, schematized_street_intervals);
        json_object.values["street_intervals"] = street_intervals;
        json_object.values["schematized_street_intervals"] = schematized_street_intervals;

        JSON::Array json_via_points_array;
        for (auto& via_idx : via_indices)
        {
            JSON::Array json_via_coordinate;
            json_via_coordinate.values.push_back(route_info[via_idx].location.lat / COORDINATE_PRECISION);
            json_via_coordinate.values.push_back(route_info[via_idx].location.lon / COORDINATE_PRECISION);
            json_via_points_array.values.push_back(json_via_coordinate);
        }
        json_object.values["via_points"] = json_via_points_array;

        json_object.values["found_alternative"] = JSON::False();

        JSON::Array schematized_instructions;
        for (auto& info : schematized_info)
        {
            JSON::Array instruction;
            instruction.values.push_back(cast::integral_to_string(cast::enum_to_underlying(info.turn_instruction)));
            schematized_instructions.values.push_back(instruction);
        }
        json_object.values["schematized_instructions"] = schematized_instructions;
        JSON::render(reply.content, json_object);
    }

  private:
    SimplePathSchematization sps;
    std::string descriptor_string;
    DataFacadeT *facade;
};

#endif // VIA_ROUTE_PLUGIN_H
