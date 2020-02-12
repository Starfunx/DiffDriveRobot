#ifndef GCODE_H
#define GCODE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
    uint16_t params;
    uint16_t params2;

    uint16_t N; ///< Line number reduced to 16 bit
    uint16_t M; ///< G-code M value if set
    uint16_t G; ///< G-code G value if set
    float X; ///< G-code X value if set
    float Y; ///< G-code Y value if set
    float Z; ///< G-code Z value if set
    float E; ///< G-code E value if set
    float F; ///< G-code F value if set
    int32_t S; ///< G-code S value if set
    int32_t P; ///< G-code P value if set
    float I; ///< G-code I value if set
    float J; ///< G-code J value if set
    float R; ///< G-code R value if set
    float D; ///< G-code D value if set
    float C; ///< G-code C value if set
    float H; ///< G-code H value if set
    float A; ///< G-code A value if set
    float B; ///< G-code B value if set
    float K; ///< G-code K value if set
    float L; ///< G-code L value if set
    float O; ///< G-code O value if set

    char *text; ///< Text message of g-code if present.
    //moved the byte to the end and aligned ints on short boundary
    // Old habit from PC, which require alignments for data types such as int and long to be on 2 or 4 byte boundary
    // Otherwise, the compiler adds padding, wasted space.
    uint8_t T; // This may not matter on any of these controllers, but it can't hurt
    // True if origin did not come from serial console. That way we can send status messages to
    // a host only if he would normally not know about the mode switch.
    bool internalCommand;
}gcodeCommand_context;


bool hasM(gcodeCommand_context* gcodeCommand);
bool hasN(gcodeCommand_context* gcodeCommand);
bool hasG(gcodeCommand_context* gcodeCommand);
bool hasX(gcodeCommand_context* gcodeCommand);
bool hasY(gcodeCommand_context* gcodeCommand);
bool hasZ(gcodeCommand_context* gcodeCommand);
bool hasNoXYZ(gcodeCommand_context* gcodeCommand);
bool hasE(gcodeCommand_context* gcodeCommand);
bool hasF(gcodeCommand_context* gcodeCommand);
bool hasT(gcodeCommand_context* gcodeCommand);
bool hasS(gcodeCommand_context* gcodeCommand);
bool hasP(gcodeCommand_context* gcodeCommand);
bool hasString(gcodeCommand_context* gcodeCommand);
bool hasI(gcodeCommand_context* gcodeCommand);
bool hasJ(gcodeCommand_context* gcodeCommand);
bool hasR(gcodeCommand_context* gcodeCommand);
bool hasD(gcodeCommand_context* gcodeCommand);
bool hasC(gcodeCommand_context* gcodeCommand);
bool hasH(gcodeCommand_context* gcodeCommand);
bool hasA(gcodeCommand_context* gcodeCommand);
bool hasB(gcodeCommand_context* gcodeCommand);
bool hasK(gcodeCommand_context* gcodeCommand);
bool hasL(gcodeCommand_context* gcodeCommand);
bool hasO(gcodeCommand_context* gcodeCommand);
long getS(gcodeCommand_context* gcodeCommand, long def);
long getP(gcodeCommand_context* gcodeCommand, long def);
void setFormatError(gcodeCommand_context* gcodeCommand);


bool gcode_parseAscii(gcodeCommand_context* gcodeCommand, char *line);
float parseFloatValue(char *s);
long parseLongValue(char *s);

#endif
