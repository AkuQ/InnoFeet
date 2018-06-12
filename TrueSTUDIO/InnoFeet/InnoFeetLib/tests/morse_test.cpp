
#include "gmock/gmock.h"
extern "C" {
    #include "morse.h"
}

using namespace testing;

TEST(basic_check, test_eq){
    int o = 1, _ = 1, ooo = 3, ___ = 3, _______ = 7;
    int* morse;
    int len = text_to_morse(&morse, (char*)"HELLO WORLD");
//    while((*morse) != -1) {
//        int beep = (*morse)++;
//        printf("%d\n", beep);
//    }
    int expected[] = {
        o,_,o,_,o,_,o, ___,
        o, ___,
        o,_,ooo,_,o,_,o, ___,
        o,_,ooo,_,o,_,o, ___,
        ooo,_,ooo,_,ooo,
        _______,
        o,_,ooo,_,ooo, ___,
        ooo,_,ooo,_,ooo, ___,
        o,_,ooo,_,o, ___,
        o,_,ooo,_,o,_,o, ___,
        ooo,_,o,_,o, ___
    };

    ASSERT_THAT(std::vector<int>(morse, morse + len), ElementsAreArray(expected));
}