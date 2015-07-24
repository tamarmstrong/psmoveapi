
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/



#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "psmove.h"
#include "../../src/psmove_private.h"

enum TestMode {
    NO_UPDATES,
    ALL_UPDATES,
    SMART_UPDATES,
    STATIC_UPDATES,
};

static const char *testmode_names[] = {
    "NO_UPDATES",
    "ALL_UPDATES",
    "SMART_UPDATES",
    "STATIC_UPDATES",
};

#define ITERATIONS 1000

FILE *csv;

void test_read_performance(PSMove *move, enum TestMode mode) {
    int round, max_rounds = 5;
    float sum = 0.;

    /* Only enable rate limiting if we test for smart updates */
    psmove_set_rate_limiting(move, mode == SMART_UPDATES);

    if (mode == STATIC_UPDATES) {
        psmove_set_leds(move, 0, 255, 0);
    }

    for (round=0; round<max_rounds; round++) {
        long packet_loss = 0;
        PSMove_timestamp ts_started = psmove_timestamp();
        long reads = 0;
        int old_sequence = -1, sequence;

        while (reads < ITERATIONS) {
            if (mode != NO_UPDATES) {
                if (mode != STATIC_UPDATES) {
                    psmove_set_leds(move, reads%255, reads%255, reads%255);
                }

                psmove_update_leds(move);
            }

            while ((sequence = psmove_poll(move)) == 0);

            if ((old_sequence > 0) && ((old_sequence % 16) != (sequence - 1))) {
                packet_loss++;
            }
            old_sequence = sequence;

            reads++;
        }
        PSMove_timestamp ts_finished = psmove_timestamp();
        float diff = (float)psmove_timestamp_value(psmove_timestamp_diff(ts_finished, ts_started));

        float reads_per_second = (float)reads / diff;
        printf("%ld reads in %.2f ms = %.5f reads/sec "
                "(%ldx seq jump = %.2f %%)\n",
                reads, diff*1000., reads_per_second, packet_loss,
                100. * (double)packet_loss / (double)reads);
        sum += reads_per_second;

        fprintf(csv, "%s,%.10f,%ld,%ld\n",
                testmode_names[mode],
                diff,
                reads,
                packet_loss);
    }

    printf("=====\n");
    printf("Mean over %d rounds: %f reads/sec\n", max_rounds,
            sum/(double)max_rounds);
}

int main(int argc, char* argv[])
{
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
		fprintf(stderr, "PS Move API init failed (wrong version?)\n");
		exit(1);
	}

    PSMove *move = psmove_connect();

    if (move == NULL) {
        printf("Could not connect to default Move controller.\n"
               "Please connect one via Bluetooth.\n");
        exit(1);
    }

    csv = psmove_file_open("read_performance.csv", "w");
    assert(csv != NULL);
    fprintf(csv, "mode,time,reads,dropped\n");

    printf("\n -- PS Move API Sensor Reading Performance Test -- \n");

    printf("\nTesting STATIC READ performance (non-changing LED setting)\n");
    test_read_performance(move, STATIC_UPDATES);

    printf("\nTesting SMART READ performance (rate-limited LED setting)\n");
    test_read_performance(move, SMART_UPDATES);

    printf("\nTesting BAD READ performance (continous LED setting)\n");
    test_read_performance(move, ALL_UPDATES);

    printf("\nTesting RAW READ performance (no LED setting)\n");
    test_read_performance(move, NO_UPDATES);

    printf("\n");

    psmove_file_close(csv);

	psmove_shutdown();

    return 0;
}


