
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "morse.h"

const char *morse_table[] = {
        ['A'] = ".-",
        ['B'] = "-...",
        ['C'] = "-.-.",
        ['D'] = "-..",
        ['E'] = ".",
        ['F'] = "..-.",
        ['G'] = "--.",
        ['H'] = "....",
        ['I'] = "..",
        ['J'] = ".---",
        ['K'] = "-.-.",
        ['L'] = ".-..",
        ['M'] = "--",
        ['N'] = "-.",
        ['O'] = "---",
        ['P'] = ".--.",
        ['Q'] = "--.-",
        ['R'] = ".-.",
        ['S'] = "...",
        ['T'] = "-",
        ['U'] = "..-",
        ['W'] = "...-",
        ['W'] = ".--",
        ['X'] = "-..-",
        ['Y'] = "-.--",
        ['Z'] = "--..",
        ['0'] = "-----",
        ['1'] = ".----",
        ['2'] = "..---",
        ['3'] = "...--",
        ['4'] = "....-",
        ['5'] = ".....",
        ['6'] = "-....",
        ['7'] = "--...",
        ['8'] = "---..",
        ['9'] = "----.",
        [' '] = " ",
        ['.'] = ".-.-.-",
        [255] = NULL
};

char *get_morse_char(char c) {
    int i = (int) c;
    char *ret = malloc(sizeof morse_table[i]);
    strcpy(ret, morse_table[i]);
    return ret;
}


int text_to_morse(int** buffer, char *text) {
    int dot = 1, dash = 3, space = 7;
    char *text_it;

    int morse_len = 0;
    for (text_it = text; *text_it != '\0'; text_it++) {
        char *morse_char = malloc(sizeof morse_table[(int) *text_it]);
        strcpy(morse_char, morse_table[(int) *text_it]);

        if (morse_char == NULL || morse_char[0] ==  ' ' )
            continue;
        else
            morse_len += strlen(morse_char) * 2; //
    }
    *buffer = malloc(morse_len * sizeof(int));

    int i = -1;
    for (text_it = text; *text_it != '\0'; text_it++) {
        char *morse_char = get_morse_char(*text_it);

        for (char *mit = morse_char; *mit != '\0'; mit++) {
            if (*mit == '.') { printf(".");
                i++;
                (*buffer)[i] = dot;
            } else if (*mit == '-') { printf("-");
                i++;
                (*buffer)[i] = dash;
            } else if (*mit == ' ' && i != -1) { printf("\n");
                (*buffer)[i] = space;
               goto end_word;
            } else {}
            i++;
            (*buffer)[i] = dot;
        }
        //end char:
        (*buffer)[i] = dash; printf(" ");
        continue;
        end_word:
        (*buffer)[i] = space;
    }
    return morse_len;
}

