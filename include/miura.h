/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2013, Daniel M. Lofaro <dan@danlofaro.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MIURA_PRIMARY_H
#define MIURA_PRIMARY_H


#ifdef __cplusplus
extern "C" {
#endif

//888888888888888888888888888888888888888888
//---------[Prerequisites for ACH]----------
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
//888888888888888888888888888888888888888888






//888888888888888888888888888888888888888888
//-----[Static Definitions and Offsets]-----
//888888888888888888888888888888888888888888

#define		MIURA_CHAN_REF_NAME         "miura-ref"                    ///> ref ach channel
#define		MIURA_CHAN_STATE_NAME       "miura-state"                  ///>  state ach channel
#define         MIURA_LOOP_PERIOD         0.005  ///> period for main loopin sec (0.005 = 200hz)
#define         MIURA_JOINT_COUNT         6
#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */

#define OFF 0 // off static
#define ON  1 // on static

typedef struct miura_joint{
    double ref;
    double pos;
}__attribute__((packed)) miura_joint_t;


typedef struct miura_state {
    miura_joint_t joint[MIURA_JOINT_COUNT];
}__attribute__((packed)) miura_state_t;

typedef struct miura_ref {
    miura_joint_t joint[MIURA_JOINT_COUNT];
}__attribute__((packed)) miura_ref_t;


extern int miura_debug;


#ifdef __cplusplus
}
#endif



#endif //HUBO_PRIMARY_H

