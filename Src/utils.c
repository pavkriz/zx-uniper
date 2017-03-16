/*
 * utils.h
 *
 *  Created on: 15. 3. 2017
 *      Author: mazlik
 */

#include "utils.h"

#include <string.h>
#include <ctype.h>

void strtrim(char *str) {
    char *start, *end;

    /* Find first non-whitespace */
    for (start = str; *start; start++)
    {
        if (!isspace((unsigned char)start[0]))
            break;
    }

    /* Find start of last all-whitespace */
    for (end = start + strlen(start); end > start + 1; end--)
    {
        if (!isspace((unsigned char)end[-1]))
            break;
    }

    *end = 0; /* Truncate last whitespace */

    /* Shift from "start" to the beginning of the string */
    if (start > str)
        memmove(str, start, (end - start) + 1);
}

/**
 * Copy string from src to dst (max size_of_dst chars).
 * Right-pad dst with spaces up to size_of_dst total chars.
 * Dst will not be NULL-terminated!
 */
void copy_right_padded(char *dst, char *src, int size_of_dst) {
	strlcpy(dst, src, size_of_dst);
	if (strlen(src) < size_of_dst) {
		memset(dst + strlen(src), 32, size_of_dst - strlen(src));
	}
}

/*
 * Swap even and odd bytes in non-NULL-terminated string (actually space-padded) in order to
 * have the order expected in ATA IDENTIFY string buffers.
 * Size should be even number.
 */
void scramble_ata_string(char *str, int size) {
	int i;
	char tmp;
	for (i = 0; i < size; i+=2) {
		tmp = str[i];
		str[i] = str[i+1];
		str[i+1] = tmp;
	}
}
