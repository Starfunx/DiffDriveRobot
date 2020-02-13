#include "gcode.h"

bool gcode_parseAscii(gcodeCommand_context* gcodeCommand, char *line)
{
    char *pos = line;
    gcodeCommand->params = 0;
    gcodeCommand->params2 = 0;
	bool hasChecksum = false;
    char c;
    while ( (c = *(pos++)) )
    {
        if(c == '(' || c == '%') break; // alternative comment or program block
        switch(c)
        {
        case 'N':
        case 'n':
        {
            gcodeCommand->N = parseLongValue(pos) & 0xffff;
            gcodeCommand->params |=1;
            break;
        }
        case 'G':
        case 'g':
        {
            gcodeCommand->G = parseLongValue(pos) & 0xffff;
            gcodeCommand->params |= 4;
            break;
        }
        case 'M':
        case 'm':
        {
            gcodeCommand->M = parseLongValue(pos) & 0xffff;
            gcodeCommand->params |= 2;
            break;
        }
        case 'X':
        case 'x':
        {
            gcodeCommand->X = parseFloatValue(pos);
            gcodeCommand->params |= 8;
            break;
        }
        case 'Y':
        case 'y':
        {
            gcodeCommand->Y = parseFloatValue(pos);
            gcodeCommand->params |= 16;
            break;
        }
        case 'Z':
        case 'z':
        {
            gcodeCommand->Z = parseFloatValue(pos);
            gcodeCommand->params |= 32;
            break;
        }
        case 'E':
        case 'e':
        {
            gcodeCommand->E = parseFloatValue(pos);
            gcodeCommand->params |= 64;
            break;
        }
        case 'F':
        case 'f':
        {
            gcodeCommand->F = parseFloatValue(pos);
            gcodeCommand->params |= 256;
            break;
        }
        case 'T':
        case 't':
        {
            gcodeCommand->T = parseLongValue(pos) & 0xff;
            gcodeCommand->params |= 512;
            break;
        }
        case 'S':
        case 's':
        {
            gcodeCommand->S = parseLongValue(pos);
            gcodeCommand->params |= 1024;
            break;
        }
        case 'P':
        case 'p':
        {
            gcodeCommand->P = parseFloatValue(pos);
            gcodeCommand->params |= 2048;
            break;
        }
        case 'I':
        case 'i':
        {
            gcodeCommand->I = parseFloatValue(pos);
            gcodeCommand->params2 |= 1;
            break;
        }
        case 'J':
        case 'j':
        {
            gcodeCommand->J = parseFloatValue(pos);
            gcodeCommand->params2 |= 2;
            break;
        }
        case 'R':
        case 'r':
        {
            gcodeCommand->R = parseFloatValue(pos);
            gcodeCommand->params2 |= 4;
            break;
        }
        case 'D':
        case 'd':
        {
            gcodeCommand->D = parseFloatValue(pos);
            gcodeCommand->params2 |= 8;
            break;
        }
        case 'C':
        case 'c':
        {
	        gcodeCommand->C = parseFloatValue(pos);
	        gcodeCommand->params2 |= 16;
	        break;
        }
        case 'H':
        case 'h':
        {
	        gcodeCommand->H = parseFloatValue(pos);
	        gcodeCommand->params2 |= 32;
	        break;
        }
        case 'A':
        case 'a':
        {
	        gcodeCommand->A = parseFloatValue(pos);
	        gcodeCommand->params2 |= 64;
	        break;
        }
        case 'B':
        case 'b':
        {
	        gcodeCommand->B = parseFloatValue(pos);
	        gcodeCommand->params2 |= 128;
	        break;
        }
        case 'K':
        case 'k':
        {
	        gcodeCommand->K = parseFloatValue(pos);
	        gcodeCommand->params2 |= 256;
	        break;
        }
        case 'L':
        case 'l':
        {
	        gcodeCommand->L = parseFloatValue(pos);
	        gcodeCommand->params2 |= 512;
	        break;
        }
        case 'O':
        case 'o':
        {
	        gcodeCommand->O = parseFloatValue(pos);
	        gcodeCommand->params2 |= 1024;
	        break;
        }
        case '*' : //checksum
        {
            break;
        }
        default:
            break;
        }// end switch
    }// end while
    return true;
}



bool hasM(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 2)!=0);
}
bool hasN(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 1)!=0);
}
bool hasG(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 4)!=0);
}
bool hasX(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 8)!=0);
}
bool hasY(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 16)!=0);
}
bool hasZ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 32)!=0);
}
bool hasNoXYZ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 56)==0);
}
bool hasE(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 64)!=0);
}
bool hasF(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 256)!=0);
}
bool hasT(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 512)!=0);
}
bool hasS(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 1024)!=0);
}
bool hasP(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 2048)!=0);
}
bool hasString(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params & 32768)!=0);
}
bool hasI(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 1)!=0);
}
bool hasJ(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 2)!=0);
}
bool hasR(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 4)!=0);
}
bool hasD(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 8)!=0);
}
bool hasC(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 16)!=0);
}
bool hasH(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 32)!=0);
}
bool hasA(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 64)!=0);
}
bool hasB(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 128)!=0);
}
bool hasK(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 256)!=0);
}
bool hasL(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 512)!=0);
}
bool hasO(gcodeCommand_context* gcodeCommand)
{
    return ((gcodeCommand->params2 & 1024)!=0);
}

long getS(gcodeCommand_context* gcodeCommand, long def)
{
    return (hasS(gcodeCommand) ? gcodeCommand->S : def);
}
long getP(gcodeCommand_context* gcodeCommand, long def)
{
    return (hasP(gcodeCommand) ? gcodeCommand->P : def);
}

void setFormatError(gcodeCommand_context* gcodeCommand) {
    gcodeCommand->params2 |= 32768;
}


float parseFloatValue(char *s)
{
    char *endPtr;
    while(*s == 32) s++; // skip spaces
    float f = (strtod(s, &endPtr));
    if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
    return f;
}

long parseLongValue(char *s)
{
    char *endPtr;
    while(*s == 32) s++; // skip spaces
    long l = (strtol(s, &endPtr, 10));
    if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
    return l;
}
