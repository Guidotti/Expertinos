/*
Copyright (c) 2000-2003, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*! \file mainCoach.cpp
<pre>
<b>File:</b>          mainCoach.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       28/11/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the main of the program that is used
                      to start the coach. It creates
                      all classes, starts the different threads and calls the
                      mainloop. Furthermore it parses the incoming arguments
                      to adjust the variables.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
28/11/2000       Jelle Kok       Initial version created
</pre>
*/

#include"ActHandler.h"
#include"SenseHandler.h"
#include"BasicCoach.h"
#include"Parse.h"

#include<string.h>   // strcpy
#ifdef WIN32
  #include <windows.h>  // needed for CreateThread
#else
  #include <pthread.h>  // needed for pthread_create
#endif
#include<stdlib.h>   // exit

extern Logger Log; /*!< This is a reference to the Logger to write log info to*/
void printOptions( );


/*! This is the main function and creates and links all the different
    classes.  First it reads in all the parameters from the command
    prompt (<program name> -help)and uses these values to create the
    classes. After all the classes are linked, the mainLoop in the
    Player class is called. */
int main( int argc, char * argv[] )
{
#ifdef WIN32
  HANDLE         listen, sense;
#else
  pthread_t      listen, sense;
#endif
  ServerSettings ss;
  PlayerSettings cs;

  // define variables for command options and initialize with default values
  char     strTeamName[MAX_TEAM_NAME_LENGTH] = "UvA_Trilearn";
  int      iPort                             = 6002;
  int      iMinLogLevel                      = 0 ;
  int      iMaxLogLevel                      = 0;
  char     strHost[128]                      = "localhost";
  double   dVersion                          = 14;
  int      iMode                             = 0;
  char     strFormations[128]                = "formations.conf";
  int      iNr                               = 0;
  int      iReconnect                        = -1;
  bool     bInfo                             = false;
  bool     bSuppliedLogFile                  = false;
  ofstream os;

  // read in all the command options and change the associated variables
  // assume every two values supplied at prompt, form a duo
  char * str;
  for( int i = 1 ; i < argc ; i = i + 2  )
  {
    // help is only option that does not have to have an argument
    if( i + 1 >= argc && strncmp( argv[i], "-help", 3 ) != 0 )
    {
      cout << "Need argument for option: " << argv[i] << endl;
      exit( 0 );
    }
    // read a command option
    if( argv[i][0] == '-' && strlen( argv[i] ) > 1)
    {
      switch( argv[i][1] )
      {
        case 'h':                                   // host server or help
          if( strlen( argv [i]) > 2 && argv[i][2] == 'e' )
          {
            printOptions( );
            exit(0);
          }
          else
            strcpy( strHost, argv[i+1] );
          break;
        case 'f':                                   // formations file
          strcpy( strFormations, argv[i+1] );
          break;
        case 'c':                                   // clientconf file
          if( cs.readValues( argv[i+1], ":" ) == false )
            cerr << "Error in reading client file: " << argv[i+1] << endl;
          break;
        case 'i':                                   // info 1 0
          str   = &argv[i+1][0];
          bInfo = (Parse::parseFirstInt( &str ) == 1 ) ? true : false ;
          break;
        case 'l':                                   // loglevel int[..int]
          str = &argv[i+1][0];
          iMinLogLevel = Parse::parseFirstInt( &str );
          while( iMinLogLevel != 0 )
          {
            if( *str == '.' ) // '.' indicates range of levels
            {
              iMaxLogLevel = Parse::parseFirstInt( &str );
              if( iMaxLogLevel == 0 ) iMaxLogLevel = iMinLogLevel;
              Log.addLogRange( iMinLogLevel, iMaxLogLevel );
            }
            else
              Log.addLogLevel( iMinLogLevel );
            iMinLogLevel = Parse::parseFirstInt( &str );
          }
          break;
        case 'm':                                   // mode int
          str = &argv[i+1][0];
          iMode = Parse::parseFirstInt( &str );
          break;
        case 'o':                                   // output file log info
          os.open( argv[i+1] );
          bSuppliedLogFile = true;
          break;
        case 'p':                                   // port
          str = &argv[i+1][0];
          iPort = Parse::parseFirstInt( &str );
          break;
        case 's':                                   // serverconf file
          if( ss.readValues( argv[i+1], ":" ) == false )
            cerr << "Error in reading server file: " << argv[i+1] << endl;
          break;
        case 't':                                   // teamname name
          strcpy( strTeamName, argv[i+1] );
          break;
        case 'v':                                   // version version
          str = &argv[i+1][0];
          dVersion = Parse::parseFirstDouble( &str );
          break;
        default:
          cerr << "(main) Unknown command option: " << argv[i] << endl;
      }
    }
  }


  if( bInfo == true )
  cout << "team         : "  << strTeamName    << endl <<
          "port         : "  << iPort          << endl <<
          "host         : "  << strHost        << endl <<
          "version      : "  << dVersion       << endl <<
          "min loglevel : "  << iMinLogLevel   << endl <<
          "max loglevel : "  << iMaxLogLevel   << endl <<
          "mode         : "  << iMode          << endl <<
          "playernr     : "  << iNr            << endl <<
          "reconnect    : "  << iReconnect     << endl ;

  if( bSuppliedLogFile == true )
    Log.setOutputStream( os );                   // initialize logger
  else
    Log.setOutputStream( cout );
  Log.restartTimer( );
  Formations fs( strFormations, (FormationT)cs.getInitialFormation(), iNr );
                                               // read formations file
  WorldModel wm( &ss, &cs, &fs );              // create worldmodel
  Connection c( strHost, iPort, MAX_MSG );     // make connection with server
  ActHandler a( &c, &wm, &ss );                // link actHandler and WM
  SenseHandler s( &c, &wm, &ss, &cs );         // link senseHandler with wm
  bool isTrainer = (iPort == ss.getCoachPort()) ? true : false;
  BasicCoach bp( &a, &wm, &ss, strTeamName, dVersion, isTrainer );
                                               // link acthandler and WM

#ifdef WIN32
  DWORD id1;
  sense = CreateThread(NULL, 0, &sense_callback, &s, 0, &id1);
  if (sense == NULL)
  {
      cerr << "create thread error" << endl;
      return false;
  }
#else
  pthread_create( &sense, NULL, sense_callback  , &s); // start listening
#endif

  if( iMode > 0 && iMode < 9 )       // only listen to sdtdin when not playing
#ifdef WIN32
  {
    DWORD id2;
    listen = CreateThread(NULL, 0, &stdin_callback, &bp, 0, &id2);
    if ( listen == NULL)
    {
        cerr << "create thread error" << endl;
        return false;
    }
  }
#else
    pthread_create( &listen, NULL, stdin_callback, &bp);
#endif
  
  if( iMode == 0 )
    bp.mainLoopNormal();

  c.disconnect();
  
  os.close();
}


/*! This function prints the command prompt options that can be supplied to the
    program. */
void printOptions( )
{
  cout << "Command options:"                                        << endl <<
   " c(lientconf) file     - use file as client conf file"           << endl <<
   " f(ormations) file     - file with formation info"               << endl <<
   " he(lp)                - print this information"                 << endl <<
   " h(ost) hostname       - host to connect with"                   << endl <<
   " i(nfo) 0/1            - print variables used to start"          << endl <<
   " l(oglevel) int[..int] - level of debug info"                    << endl <<
   " m(ode) int            - which mode to start up with"            << endl <<
   " o(utput) file         - write log info to (screen is default)"  << endl <<
   " p(ort)                - port number to connect with"            << endl <<
   " s(erverconf) file     - use file as server conf file"           << endl <<
   " t(eamname) name       - name of your team"                      << endl;
}


