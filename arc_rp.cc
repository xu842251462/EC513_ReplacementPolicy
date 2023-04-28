/**
 * Copyright (c) 2018-2020 Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/replacement_policies/arc_rp.hh"

#include <cassert>
#include <memory>
#include <vector>
#include <algorithm>
#include "mem/packet.hh"
#include "params/ARCRP.hh"
#include "sim/cur_tick.hh"

#define c 8
#define index_size 32

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(ReplacementPolicy, replacement_policy);
namespace replacement_policy
{

ARC::ARC(const Params &p)
  : Base(p)
{
    pp = 0;
}

void
ARC::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    std::static_pointer_cast<ARCReplData>(
        replacement_data)->lastTouchTick = Tick(0);
}
void

ARC::touch(const std::shared_ptr<ReplacementData>& replacement_data)
    const
{
    panic("Cant train ARC's predictor without access information.");
}
void
ARC::touch(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    
    // Update last touch timestamp
    std::static_pointer_cast<ARCReplData>(
        replacement_data)->lastTouchTick = curTick();
    std::static_pointer_cast<ARCReplData>(
        replacement_data)->pkt = pkt;

   std::vector<std::shared_ptr<ReplacementData>>::iterator itt1;
   std::vector<std::shared_ptr<ReplacementData>>::iterator itt2;

    itt1 = std::find_if(t1.begin(), t1.end(),
        [pkt](std::shared_ptr<ReplacementData> obj) { return std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr()>>6 == pkt->getAddr()>>6; });

    if(itt1 != t1.end()){
        t2.insert(t2.begin(),replacement_data);
        t1.erase(itt1);
    }
}

void
ARC::reset(const std::shared_ptr<ReplacementData>& replacement_data)
    const
{
    panic("Cant train ARC's predictor without access information.");
}

void
ARC::reset(const std::shared_ptr<ReplacementData>& replacement_data,
    const PacketPtr pkt)
{
    // Set last touch timestamp
    int set = (pkt->getAddr()>>6)%index_size;

    int t1size = std::count_if(t1.begin(), t1.end(), 
                          [set](std::shared_ptr<ReplacementData> obj) { 
                              return (std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() >>6)%index_size == set; 
                          });
    
    int t2size = std::count_if(t2.begin(), t2.end(), 
                          [set](std::shared_ptr<ReplacementData> obj) { 
                              return (std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() >>6)%index_size == set; 
                          });
    
    int b1size = std::count_if(b1.begin(), b1.end(), 
                          [set](std::shared_ptr<ReplacementData> obj) { 
                              return (std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() >>6)%index_size == set; 
                          });

    int b2size = std::count_if(b2.begin(), b2.end(), 
                          [set](std::shared_ptr<ReplacementData> obj) { 
                              return (std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() >>6)%index_size == set; 
                          });


    std::static_pointer_cast<ARCReplData>(
        replacement_data)->lastTouchTick = curTick();
    std::static_pointer_cast<ARCReplData>(
        replacement_data)->pkt = pkt;

    std::vector<std::shared_ptr<ReplacementData>>::iterator itb1;
    std::vector<std::shared_ptr<ReplacementData>>::iterator itb2;


    itb1 = std::find_if(b1.begin(), b1.end(),
        [pkt](std::shared_ptr<ReplacementData> obj) { return std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() == pkt->getAddr(); });
    itb2 = std::find_if(b2.begin(), b2.end(),
        [pkt](std::shared_ptr<ReplacementData> obj) { return std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() == pkt->getAddr(); });

    if(itb1 != b1.end()){
        float delta=0;
        if (b1size>b2size)
            delta = 1;
        else    
            delta = b2size/b1size;
        pp = std::min((float)c,pp+delta);

        t2.insert(t2.begin(),replacement_data);
        b1.erase(itb1);
        state = 0;
    }
    else if(itb2 != b2.end()){
        float delta=0;
        if (b2size>b1size)
            delta = 1;
        else    
            delta = b1size/b2size;
        pp = std::max((float)0,pp-delta);

        t2.insert(t2.begin(),replacement_data);
        b2.erase(itb2);
        state = 1;
    }
    else{
        if(t1size+b1size == c){
            if(t1size<c){
                 auto smallest = std::min_element(b1.begin(), b1.end(), 
                 [](std::shared_ptr<ReplacementData> a, std::shared_ptr<ReplacementData> b) {
                    return std::static_pointer_cast<ARCReplData>(a)->lastTouchTick < std::static_pointer_cast<ARCReplData>(b)->lastTouchTick;
                 });
                 b1.erase(smallest);
            }
            else{
                auto smallest = std::min_element(t1.begin(), t1.end(), 
                 [](std::shared_ptr<ReplacementData> a, std::shared_ptr<ReplacementData> b) {
                    return std::static_pointer_cast<ARCReplData>(a)->lastTouchTick < std::static_pointer_cast<ARCReplData>(b)->lastTouchTick;
                 });
                 t1.erase(smallest);
            }
        }
        else{
            if(t1size+t2size+b1size+b2size>=c){
                auto smallest = std::min_element(b2.begin(), b2.end(), 
                 [](std::shared_ptr<ReplacementData> a, std::shared_ptr<ReplacementData> b) {
                    return std::static_pointer_cast<ARCReplData>(a)->lastTouchTick < std::static_pointer_cast<ARCReplData>(b)->lastTouchTick;
                 });
                b2.erase(smallest);
            }
        }
        t1.insert(t1.begin(),replacement_data);
        state = 0;
    }

    t1size = std::count_if(t1.begin(), t1.end(), 
                          [set](std::shared_ptr<ReplacementData> obj) { 
                              return (std::static_pointer_cast<ARCReplData>(obj)->pkt->getAddr() >>6)%index_size == set; 
                          });
    

    if(t1size>pp || (state==1 && t1size==pp)){
        auto smallest = std::min_element(t1.begin(), t1.end(), 
        [](std::shared_ptr<ReplacementData> a, std::shared_ptr<ReplacementData> b) {
            return std::static_pointer_cast<ARCReplData>(a)->lastTouchTick < std::static_pointer_cast<ARCReplData>(b)->lastTouchTick;
        });
        
        b1.insert(b1.begin(),*smallest);
        t1.erase(smallest);
        Vict = *smallest;
       // victim = smallest;
    }
    else{
        auto smallest = std::min_element(t2.begin(), t2.end(), 
        [](std::shared_ptr<ReplacementData> a, std::shared_ptr<ReplacementData> b) {
            return std::static_pointer_cast<ARCReplData>(a)->lastTouchTick < std::static_pointer_cast<ARCReplData>(b)->lastTouchTick;
        });
        b2.insert(b2.begin(),*smallest);
        t2.erase(smallest);
        Vict =*smallest;
        
     //   victim = smallest;
    }

}

ReplaceableEntry*
ARC::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (const auto& candidate : candidates) {
        // Update victim entry if necessary
        if (candidate->replacementData == Vict) {
            victim = candidate;
        }
    }
    return victim;
}

std::shared_ptr<ReplacementData>
ARC::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new ARCReplData());
}

} // namespace replacement_policy
} // namespace gem5
