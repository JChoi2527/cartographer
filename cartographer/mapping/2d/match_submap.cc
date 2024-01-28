#include "match_submap.h"

// mapping/internal/constraints/constraint_builder_2d 파일 참고

MatchSubmap::MatchSubmap() {}

void MatchSubmap::setFullMatchSubmap(bool state) {
    s_full_match_submap_ = state;

    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;
    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;
    std::cout << "full_match_submap : " << s_full_match_submap_ << std::endl;

    if(!s_is_first_match_done) { // only one time.
        std::cout << "Full Match Submap Occured for the first time!" << std::endl;
        s_is_first_match_done = true;
    }
}

bool MatchSubmap::getFirstFullMatchSubmap() {
    if(s_is_first_match_done) {
        return true;
    }

    return false;
}