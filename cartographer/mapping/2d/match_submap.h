#ifndef MATCH_SUBMAP_AMRLABS_H
#define MATCH_SUBMAP_AMRLABS_H

#include <iostream>
#include <chrono>
#include <thread>

class MatchSubmap {
private:
    bool is_limit_second_done = false;
    int limit_second = 50;

public:
    MatchSubmap();
    static void setFullMatchSubmap(bool state);
    static void setLocalMatchSubmap(bool state);
    static bool getFirstMatchSubmap();

    bool delayTime();
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::duration duration_time;
    
    static bool s_full_match_submap_;
    static bool s_local_match_submap_;
    static int s_inserted_submap_num;
    static bool s_is_first_match_done;

};

#endif
