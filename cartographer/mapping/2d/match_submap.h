#ifndef MATCH_SUBMAP_AMRLABS_H
#define MATCH_SUBMAP_AMRLABS_H

#include <iostream>

class MatchSubmap {

public:
    MatchSubmap();
    static void setFullMatchSubmap(bool state);
    static bool getFirstFullMatchSubmap();
    
    static bool s_full_match_submap_;
    static int s_inserted_submap_num;
    static bool s_is_first_match_done;

};

#endif
