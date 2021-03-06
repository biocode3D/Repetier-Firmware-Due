/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#include "Repetier.h"

#ifndef FEATURE_CHECKSUM_FORCED
#define FEATURE_CHECKSUM_FORCED false
#endif

GCode GCode::commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
byte GCode::bufferReadIndex=0; ///< Read position in gcode_buffer.
byte GCode::bufferWriteIndex=0; ///< Write position in gcode_buffer.
byte GCode::commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
byte GCode::commandsReceivingWritePosition=0; ///< Writing position in gcode_transbuffer.
byte GCode::sendAsBinary; ///< Flags the command as binary input.
byte GCode::wasLastCommandReceivedAsBinary=0; ///< Was the last successful command in binary mode?
byte GCode::commentDetected=false; ///< Flags true if we are reading the comment part of a command.
byte GCode::binaryCommandSize; ///< Expected size of the incoming binary command.
bool GCode::waitUntilAllCommandsAreParsed=false; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
long GCode::lastLineNumber=0; ///< Last line number received.
long GCode::actLineNumber; ///< Line number of current command.
char GCode::waitingForResend=-1; ///< Waiting for line to be resend. -1 = no wait.
volatile byte GCode::bufferLength=0; ///< Number of commands stored in gcode_buffer
millis_t GCode::timeOfLastDataPacket=0; ///< Time, when we got the last data packet. Used to detect missing bytes.

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcommings in the standard
RepRap communication method, while remaining backward compatible. To use the improved
features of this protocal, you need a host which speaks it. On Windows the recommended
host software is Repetier-Host. It is developed in parallel to this firmware and supports
all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce communication failures.
  The standard method is to transfer a checksum at the end of the line. This checksum is the
  XORd value of all characters send. The value is limited to a range between 0 and 127. It can
  not detect two identical missing characters or a wrong order. Therefore the new protocol
  uses Fletchers checksum, which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to less then 50% and
  it speeds up decoding the command. No slow conversion from string to floats are needed.

*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 bytes define the
data. From this bitfield, this function computes the size of the command
including the 2 bytes of the bitfield and the 2 bytes for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned byte
- G : Bit 2 :  8-Bit unsigned byte
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more bytes following with Bits, only for future versions
- Int :Bit 14 : Marks it as internal command,
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
*/
byte GCode::computeBinarySize(char *ptr)  // unsigned int bitfield) {
{
    byte s = 4; // include checksum and bitfield
    unsigned int bitfield = *(int*)ptr;
    if(bitfield & 1) s+=2;
    if(bitfield & 8) s+=4;
    if(bitfield & 16) s+=4;
    if(bitfield & 32) s+=4;
    if(bitfield & 64) s+=4;
    if(bitfield & 256) s+=4;
    if(bitfield & 512) s+=1;
    if(bitfield & 1024) s+=4;
    if(bitfield & 2048) s+=4;
    if(bitfield & 4096)   // Version 2 or later
    {
        s+=2; // for bitfield 2
        unsigned int bitfield2 = *(int*)(ptr+2);
        if(bitfield & 2) s+=2;
        if(bitfield & 4) s+=2;
        if(bitfield2 & 1) s+= 4;
        if(bitfield2 & 2) s+= 4;
        if(bitfield2 & 4) s+= 4;
        if(bitfield & 32768) s+=(byte)ptr[4]+1;
        //OUT_P_I_LN("LenStr:",(int)ptr[4]);
        //OUT_P_I_LN("LenBinV2:",s);
    }
    else
    {
        if(bitfield & 2) s+=1;
        if(bitfield & 4) s+=1;
        if(bitfield & 32768) s+=16;
    }
    return s;
}

void GCode::requestResend()
{
    HAL::serialFlush();
    commandsReceivingWritePosition=0;
    if(sendAsBinary)
        waitingForResend = 30;
    else
        waitingForResend = 14;
    Com::println();
    Com::printFLN(Com::tResend,lastLineNumber+1);
    Com::printFLN(Com::tOk);
}
/**
  Check if result is plausible. If it is, an ok is send and the command is stored in queue.
  If not, a resend and ok is send.
*/
void GCode::checkAndPushCommand()
{
    if(hasM())
    {
        if(M==110)   // Reset line number
        {
            lastLineNumber = actLineNumber;
            Com::printFLN(Com::tOk);
            return;
        }
        if(M==112)   // Emergency kill - freeze printer
        {
            Commands::emergencyStop();
        }
    }
    if(hasN())
    {
        if((((lastLineNumber+1) & 0xffff)!=(actLineNumber&0xffff)))
        {
            if(waitingForResend<0)   // after a resend, we have to skip the garbage in buffers, no message for this
            {
                if(Printer::debugErrors())
                {
                    Com::printF(Com::tExpectedLine,lastLineNumber+1);
                    Com::printFLN(Com::tGot,actLineNumber);
                }
                requestResend(); // Line missing, force resend
            }
            else
            {
                --waitingForResend;
                commandsReceivingWritePosition = 0;
                Com::printFLN(Com::tSkip,actLineNumber);
                Com::printFLN(Com::tOk);
            }
            return;
        }
        lastLineNumber = actLineNumber;
    }
    pushCommand();
#ifdef ACK_WITH_LINENUMBER
    Com::printFLN(Com::tOkSpace,actLineNumber);
#else
    Com::printFLN(Com::tOk);
#endif
    wasLastCommandReceivedAsBinary = sendAsBinary;
    waitingForResend = -1; // everything is ok.
}
void GCode::pushCommand()
{
    bufferWriteIndex = (bufferWriteIndex+1) % GCODE_BUFFER_SIZE;
    bufferLength++;
#ifndef ECHO_ON_EXECUTE
    echoCommand();
#endif
}
/**
  Get the next buffered command. Returns 0 if no more commands are buffered. For each
  returned command, the gcode_command_finished() function must be called.
*/
GCode *GCode::peekCurrentCommand()
{
    if(bufferLength==0) return 0; // No more data
    return &commandsBuffered[bufferReadIndex];
}
/** \brief Removes the last returned command from cache.

*/
void GCode::popCurrentCommand()
{
    if(!bufferLength) return; // Should not happen, but safety first
#ifdef ECHO_ON_EXECUTE
    echoCommand();
#endif
    bufferReadIndex = (bufferReadIndex+1) % GCODE_BUFFER_SIZE;
    bufferLength--;
}

void GCode::echoCommand()
{
    if(Printer::debugEcho())
    {
        Com::printF(Com::tEcho);
        printCommand();
    }
}

/** \brief Execute commands in progmem stored string. Multiple commands are seperated by \n */
void GCode::executeFString(FSTRINGPARAM(cmd))
{
    char buf[80];
    byte buflen;
    char c;
    GCode code;
    do
    {
        // Wait for a free place in command buffer
        // Scan next command from string
        byte comment=0;
        buflen = 0;
        do
        {
            c = HAL::readFlashByte(cmd++);
            if(c == 0 || c == '\n') break;
            if(c == ';') comment = 1;
            if(comment) continue;
            buf[buflen++] = c;
        }
        while(buflen<79);
        if(buflen==0)   // empty line ignore
        {
            continue;
        }
        buf[buflen]=0;
        // Send command into command buffer
        if(code.parseAscii((char *)buf,false) && (code.params & 518))   // Success
        {
            Commands::executeGCode(&code);
            Printer::defaultLoopActions();
        }
    }
    while(c);
}
/** \brief Read from serial console or sdcard.

This function is the main function to read the commands from serial console or from sdcard.
It must be called frequently to empty the incoming buffer.
*/
void GCode::readFromSerial()
{
    if(waitUntilAllCommandsAreParsed && bufferLength) return;
    waitUntilAllCommandsAreParsed=false;
    GCode *act;
    millis_t time = HAL::timeInMilliseconds();
    if(bufferLength>=GCODE_BUFFER_SIZE) return; // all buffers full
    if(!HAL::serialByteAvailable())
    {
        if((waitingForResend>=0 || commandsReceivingWritePosition>0) && time-timeOfLastDataPacket>200)
        {
            requestResend(); // Something is wrong, a started line was not continued in the last second
            timeOfLastDataPacket = time;
        }
#ifdef WAITING_IDENTIFIER
        else if(bufferLength == 0 && time-timeOfLastDataPacket>1000)   // Don't do it if buffer is not empty. It may be a slow executing command.
        {
            Com::printFLN(Com::tWait); // Unblock communication in case the last ok was not received correct.
            timeOfLastDataPacket = time;
        }
#endif
    }
    while(HAL::serialByteAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE)    // consume data until no data or buffer full
    {
        timeOfLastDataPacket = HAL::timeInMilliseconds();
        commandReceiving[commandsReceivingWritePosition++] = HAL::serialReadByte();
        // first lets detect, if we got an old type ascii command
        if(commandsReceivingWritePosition==1)
        {
            if(waitingForResend>=0 && wasLastCommandReceivedAsBinary)
            {
                if(!commandReceiving[0])
                {
                    waitingForResend--;   // Skip 30 zeros to get in sync
                }
                else waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if(!commandReceiving[0])
            {
                commandsReceivingWritePosition = 0;
                continue;
            }
            sendAsBinary = (commandReceiving[0] & 128)!=0;
        }
        if(sendAsBinary)
        {
            if(commandsReceivingWritePosition < 2 ) continue;
            if(commandsReceivingWritePosition == 5) binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if(commandsReceivingWritePosition==binaryCommandSize)
            {
                act = &commandsBuffered[bufferWriteIndex];
                if(act->parseBinary(commandReceiving,true))   // Success
                {
                    act->checkAndPushCommand();
                }
                else
                {
                    requestResend();
                }
                commandsReceivingWritePosition = 0;
                return;
            }
        }
        else     // Ascii command
        {
            char ch = commandReceiving[commandsReceivingWritePosition-1];
            if(ch == '\n' || ch == '\r' || ch == ':' || commandsReceivingWritePosition >= (MAX_CMD_SIZE - 1) )  // complete line read
            {
                commandReceiving[commandsReceivingWritePosition-1]=0;
                commentDetected = false;
                if(commandsReceivingWritePosition==1)   // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                act = &commandsBuffered[bufferWriteIndex];
                if(act->parseAscii((char *)commandReceiving,true))   // Success
                {
                    act->checkAndPushCommand();
                }
                else
                {
                    requestResend();
                }
                commandsReceivingWritePosition = 0;
                return;
            }
            else
            {
                if(ch == ';') commentDetected = true; // ignore new data until lineend
                if(commentDetected) commandsReceivingWritePosition--;
            }
        }
    }
#if SDSUPPORT
    if(!sd.sdmode || commandsReceivingWritePosition!=0)   // not reading or incoming serial command
    {
        return;
    }
    while( sd.filesize > sd.sdpos && commandsReceivingWritePosition < MAX_CMD_SIZE)    // consume data until no data or buffer full
    {
        timeOfLastDataPacket = HAL::timeInMilliseconds();
        int n = sd.file.read();
        if(n==-1)
        {
            Com::printFLN(Com::tSDReadError);
            break;
        }
        sd.sdpos++; // = file.curPosition();
        commandReceiving[commandsReceivingWritePosition++] = (byte)n;

        // first lets detect, if we got an old type ascii command
        if(commandsReceivingWritePosition==1)
        {
            sendAsBinary = (commandReceiving[0] & 128)!=0;
        }
        if(sendAsBinary)
        {
            if(commandsReceivingWritePosition < 2 ) continue;
            if(commandsReceivingWritePosition == 5) binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if(commandsReceivingWritePosition==binaryCommandSize)
            {
                act = &commandsBuffered[bufferWriteIndex];
                if(act->parseBinary(commandReceiving,false))   // Success
                {
                    pushCommand();
                }
                commandsReceivingWritePosition = 0;
                return;
            }
        }
        else
        {
            char ch = commandReceiving[commandsReceivingWritePosition-1];
            if(ch == '\n' || ch == '\r' || ch == ':' || commandsReceivingWritePosition >= (MAX_CMD_SIZE - 1) )  // complete line read
            {
                commandReceiving[commandsReceivingWritePosition-1]=0;
                commentDetected = false;
                if(commandsReceivingWritePosition==1)   // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                act = &commandsBuffered[bufferWriteIndex];
                if(act->parseAscii((char *)commandReceiving,false))   // Success
                {
                    pushCommand();
                }
                commandsReceivingWritePosition = 0;
                return;
            }
            else
            {
                if(ch == ';') commentDetected = true; // ignore new data until lineend
                if(commentDetected) commandsReceivingWritePosition--;
            }
        }
    }
    sd.sdmode = false;
    Com::printFLN(Com::tDonePrinting);
    commandsReceivingWritePosition = 0;
#endif
}

/**
  Converts a binary bytefield containing one GCode line into a GCode structure.
  Returns true if checksum was correct.
*/
bool GCode::parseBinary(byte *buffer,bool fromSerial)
{
    unsigned int sum1=0,sum2=0; // for fletcher-16 checksum
    // first do fletcher-16 checksum tests see
    // http://en.wikipedia.org/wiki/Fletcher's_checksum
    byte i=0;
    byte *p = buffer;
    byte len = binaryCommandSize-2;
    while (len)
    {
        byte tlen = len > 21 ? 21 : len;
        len -= tlen;
        do
        {
            sum1 += *p++;
            if(sum1>=255) sum1-=255;
            sum2 += sum1;
            if(sum2>=255) sum2-=255;
        }
        while (--tlen);
    }
    sum1 -= *p++;
    sum2 -= *p;
    if(sum1 | sum2)
    {
        if(Printer::debugErrors())
        {
            OUT_P_LN("Error:Binary cmd wrong checksum.");
        }
        return false;
    }
    p = buffer;
    params = *(unsigned int *)p;
    p+=2;
    byte textlen=16;
    if(isV2())
    {
        params2 = *(unsigned int *)p;
        p+=2;
        if(hasString())
            textlen = *p++;
    }
    else params2 = 0;
    if(params & 1)
    {
        actLineNumber=N=*(unsigned int *)p;
        p+=2;
    }
    if(isV2())   // Read G,M as 16 bit value
    {
        if(params & 2)
        {
            M=*(unsigned int *)p;
            p+=2;
        }
        if(params & 4)
        {
            G=*(unsigned int *)p;
            p+=2;
        }
    }
    else
    {
        if(params & 2)
        {
            M=*p++;
        }
        if(params & 4)
        {
            G=*p++;
        }
    }
    //if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
    if(params & 8)
    {
        X=*(float *)p;
        p+=4;
    }
    if(params & 16)
    {
        Y=*(float *)p;
        p+=4;
    }
    if(params & 32)
    {
        Z =*(float *)p;
        p+=4;
    }
    if(params & 64)
    {
        E=*(float *)p;
        p+=4;
    }
    if(params & 256)
    {
        F=*(float *)p;
        p+=4;
    }
    if(params & 512)
    {
        T=*p++;
    }
    if(params & 1024)
    {
        S=*(long int*)p;
        p+=4;
    }
    if(params & 2048)
    {
        P=*(long int*)p;
        p+=4;
    }
    if(hasI())
    {
        I=*(float *)p;
        p+=4;
    }
    if(hasJ())
    {
        J=*(float *)p;
        p+=4;
    }
    if(hasR())
    {
        R=*(float *)p;
        p+=4;
    }
    if(hasString())   // set text pointer to string
    {
        text = (char*)p;
        text[textlen] = 0; // Terminate string overwriting checksum
        waitUntilAllCommandsAreParsed=true; // Don't destroy string until executed
    }
    return true;
}

/**
  Converts a ascii GCode line into a GCode structure.
*/
bool GCode::parseAscii(char *line,bool fromSerial)
{
    bool has_checksum = false;
    char *pos;
    params = 0;
    params2 = 0;
    if((pos = strchr(line,'N'))!=0)   // Line number detected
    {
        actLineNumber = gcode_value_long(++pos);
        params |=1;
        N = actLineNumber & 0xffff;
    }
    if((pos = strchr(line,'M'))!=0)   // M command
    {
        M = gcode_value_long(++pos) & 0xffff;
        params |= 2;
        if(M>255) params |= 4096;
    }
    if(hasM() && (M == 23 || M == 28 || M == 29 || M == 30 || M == 32 || M == 117))
    {
        // after M command we got a filename for sd card management
        char *sp = line;
        while(*sp!='M') sp++; // Search M command
        while(*sp!=' ') sp++; // search next whitespace
        while(*sp==' ') sp++; // skip leading whitespaces
        text = sp;
        while(*sp)
        {
            if(M != 117 && (*sp==' ' || *sp=='*')) break; // end of filename reached
            sp++;
        }
        *sp = 0; // Removes checksum, but we don't care. Could also be part of the string.
        waitUntilAllCommandsAreParsed = true; // don't risk string be deleted
        params |= 32768;
    }
    else
    {
        if((pos = strchr(line,'G'))!=0)   // G command
        {
            G = gcode_value_long(++pos) & 0xffff;
            params |= 4;
            if(G>255) params |= 4096;
        }
        if((pos = strchr(line,'X'))!=0)
        {
            X = gcode_value(++pos);
            params |= 8;
        }
        if((pos = strchr(line,'Y'))!=0)
        {
            Y = gcode_value(++pos);
            params |= 16;
        }
        if((pos = strchr(line,'Z'))!=0)
        {
            Z = gcode_value(++pos);
            params |= 32;
        }
        if((pos = strchr(line,'E'))!=0)
        {
            E = gcode_value(++pos);
            params |= 64;
        }
        if((pos = strchr(line,'F'))!=0)
        {
            F = gcode_value(++pos);
            params |= 256;
        }
        if((pos = strchr(line,'T'))!=0)   // M command
        {
            T = gcode_value_long(++pos) & 0xff;
            params |= 512;
        }
        if((pos = strchr(line,'S'))!=0)   // M command
        {
            S = gcode_value_long(++pos);
            params |= 1024;
        }
        if((pos = strchr(line,'P'))!=0)   // M command
        {
            P = gcode_value_long(++pos);
            params |= 2048;
        }
        if((pos = strchr(line,'I'))!=0)
        {
            I = gcode_value(++pos);
            params2 |= 1;
            params |= 4096; // Needs V2 for saving
        }
        if((pos = strchr(line,'J'))!=0)
        {
            J = gcode_value(++pos);
            params2 |= 2;
            params |= 4096; // Needs V2 for saving
        }
        if((pos = strchr(line,'R'))!=0)
        {
            R = gcode_value(++pos);
            params2 |= 4;
            params |= 4096; // Needs V2 for saving
        }
    }
    if((pos = strchr(line,'*'))!=0)   // checksum
    {
        byte checksum_given = gcode_value_long(pos+1);
        byte checksum = 0;
        while(line!=pos) checksum ^= *line++;
#if FEATURE_CHECKSUM_FORCED
        Printer::flag0 |= PRINTER_FLAG0_FORCE_CHECKSUM;
#endif
        if(checksum!=checksum_given)
        {
            if(Printer::debugErrors())
            {
                Com::printErrorFLN(Com::tWrongChecksum);
            }
            return false; // mismatch
        }
    }
#if FEATURE_CHECKSUM_FORCED
    else
    {
        if(!fromSerial) return true;
        if(hasM() && (M == 110 || hasString()) return true;
        if(Printer::debugErrors())
        {
            Com::printErrorFLN(Com::tMissingChecksum);
        }
        return false;
    }
#endif
    return true;
}

/** \brief Print command on serial console */
void GCode::printCommand()
{
    if(hasM())
    {
        Com::print('M');
        Com::print((int)M);
        Com::print(' ');
    }
    if(hasG())
    {
        Com::print('G');
        Com::print((int)G);
        Com::print(' ');
    }
    if(hasT())
    {
        Com::print('T');
        Com::print((int)T);
        Com::print(' ');
    }
    if(hasX())
    {
        Com::printF(Com::tX,X);
    }
    if(hasY())
    {
        Com::printF(Com::tY,Y);
    }
    if(hasZ())
    {
        Com::printF(Com::tZ,Z);
    }
    if(hasE())
    {
        Com::printF(Com::tE,E,4);
    }
    if(hasF())
    {
        Com::printF(Com::tF,F);
    }
    if(hasS())
    {
        Com::printF(Com::tS,S);
    }
    if(hasP())
    {
        Com::printF(Com::tP,P);
    }
    if(hasI())
    {
        Com::printF(Com::tI,I);
    }
    if(hasJ())
    {
        Com::printF(Com::tJ,J);
    }
    if(hasR())
    {
        Com::printF(Com::tR,R);
    }
    if(hasString())
    {
        Com::print(text);
    }
    Com::println();
}
