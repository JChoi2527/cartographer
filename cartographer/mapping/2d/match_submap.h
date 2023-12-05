#ifndef MATCH_SUBMAP_AMRLABS_H
#define MATCH_SUBMAP_AMRLABS_H

#include <iostream>

class MatchSubmap {

public:
    MatchSubmap();
    static void setFullMatchSubmap(bool state);
    
    static bool s_full_match_submap_;
    static int s_inserted_submap_num;

};

#endif
