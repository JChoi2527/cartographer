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

bool MatchSubmap::delayTime() {
    this->duration_time = std::chrono::steady_clock::now() - this->start_time;
    if(this->duration_time > std::chrono::seconds{50} ) {
        // std::cout << "more then 30 seconds past.." << std::endl;
        // std::cout << "more then 30 seconds past.." << std::endl;
        return true;
    }
    return false;
}