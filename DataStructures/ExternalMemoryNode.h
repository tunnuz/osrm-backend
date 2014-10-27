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

#ifndef EXTERNAL_MEMORY_NODE_H__
#define EXTERNAL_MEMORY_NODE_H__

#include "QueryNode.h"

#include <string>

struct ExternalMemoryNode : NodeInfo
{
    ExternalMemoryNode(int lat, int lon, NodeID id, bool bollard, bool traffic_light);

    ExternalMemoryNode();

    static ExternalMemoryNode min_value();

    static ExternalMemoryNode max_value();

    bool bollard;
    bool trafficLight;
};

struct ExternalMemoryNodeSTXXLCompare
{
    using value_type = ExternalMemoryNode;
    bool operator()(const ExternalMemoryNode &left, const ExternalMemoryNode &right) const;
    value_type max_value();
    value_type min_value();
};

#endif /* EXTERNAL_MEMORY_NODE_H__ */
