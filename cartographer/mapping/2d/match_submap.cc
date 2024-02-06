#include "match_submap.h"

// mapping/internal/constraints/constraint_builder_2d 파일 참고

MatchSubmap::MatchSubmap() {
    this->start_time = std::chrono::steady_clock::now();
}

void MatchSubmap::setFullMatchSubmap(bool state) {
    s_full_match_submap_ = state;

    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;
    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;
    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;

    if(!s_is_first_match_done) { // only one time.
        std::cout << "Full Match Submap Occurred for the first time!" << std::endl;
        std::cout << "Full Match Submap Occurred for the first time!" << std::endl;
        s_is_first_match_done = true;
    }
}

void MatchSubmap::setLocalMatchSubmap(bool state) {
    s_local_match_submap_ = state;

    std::cout << "local_match_submap : " << s_local_match_submap_ << std::endl;

    if(!s_is_first_match_done) { // only one time either setFullMatchSubmap() or setLocalMatchSubmap()
        std::cout << "Local Match Submap Occurred for the first time!" << std::endl;
        std::cout << "Local Match Submap Occurred for the first time!" << std::endl;
        s_is_first_match_done = true;
    }

}


/// It is decided by ethier local match or full match
bool MatchSubmap::getFirstMatchSubmap() {
    if(s_is_first_match_done) {
        return true;
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(50)); // when result is false, delay...
    return false;
}

/// @brief  to delay time as limit_second (now 50). Once it reached at the limit_second (50), result will always be true.
/// @return 
bool MatchSubmap::delayTime() {
    if(this->is_limit_second_done) {
        return true; // no more caculating duration_time 
    }

    this->duration_time = std::chrono::steady_clock::now() - this->start_time;
    if(this->duration_time > std::chrono::seconds{this->limit_second} ) {
        std::cout << "more then " << this->limit_second << " seconds past.." << std::endl;
        this->is_limit_second_done = true;
        return true;
    }
    return false;
}