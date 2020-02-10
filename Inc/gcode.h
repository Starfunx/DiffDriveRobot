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

inline bool hasM(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 2)!=0);
}
inline bool hasN(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 1)!=0);
}
inline bool hasG(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 4)!=0);
}
inline bool hasX(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 8)!=0);
}
inline void unsetX(gcodeCommand_context* gcodeCommand) {
    gcodeCommand->params &= ~8;
}
inline bool hasY(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 16)!=0);
}
inline void unsetY(gcodeCommand_context* gcodeCommand) {
    gcodeCommand->params &= ~16;
}
inline bool hasZ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 32)!=0);
}
inline void unsetZ(gcodeCommand_context* gcodeCommand) {
    gcodeCommand->params &= ~32;
}
inline bool hasNoXYZ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 56)==0);
}
inline bool hasE(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 64)!=0);
}
inline bool hasF(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 256)!=0);
}
inline bool hasT(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 512)!=0);
}
inline bool hasS(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 1024)!=0);
}
inline bool hasP(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 2048)!=0);
}
inline bool hasString(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 32768)!=0);
}
inline bool hasI(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 1)!=0);
}
inline bool hasJ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 2)!=0);
}
inline bool hasR(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 4)!=0);
}
inline bool hasD(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 8)!=0);
}
inline bool hasC(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 16)!=0);
}
inline bool hasH(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 32)!=0);
}
inline bool hasA(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 64)!=0);
}
inline bool hasB(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 128)!=0);
}
inline bool hasK(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 256)!=0);
}
inline bool hasL(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 512)!=0);
}
inline bool hasO(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 1024)!=0);
}

inline long getS(gcodeCommand_context* gcodeCommand, long def)
{
    return (hasS(gcodeCommand) ? gcodeCommand->S : def);
}
inline long getP(gcodeCommand_context* gcodeCommand, long def)
{
    return (hasP(gcodeCommand) ? gcodeCommand->P : def);
}

inline void setFormatError(gcodeCommand_context* gcodeCommand) {
    gcodeCommand->params2 |= 32768;
}

bool gcode_parseAscii(gcodeCommand_context* gcodeCommand, char *line);
float parseFloatValue(char *s);
long parseLongValue(char *s);

#endif