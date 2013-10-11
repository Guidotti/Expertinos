#!/bin/tcsh

# This script starts the UvA_Trilearn_2003 team. When player numbers are 
# supplied before the (optional) host-name and team-name arguments only
# these players are started, otherwise all players are started.
# Usage:   start.sh <player-numbers> <host-name> <team-name>
#
# Example: start.sh                 all players default host and name
# Example: start.sh machine         all players on host 'machine'
# Example: start.sh localhost UvA   all players on localhost and name 'UvA'
# Example: start.sh 127.0.0.1 UvA   all players on 127.0.0.1  and name 'UvA'
# Example: start.sh 1 2 3 4         players 1-4 on default host and name
# Example: start.sh 1 2 remote      players 1-2 on host 'remote'
# Example: start.sh 9 10 remote UvA players 9-10 on host remote and name 'UvA'
# Example: start.sh 0               start coach on default host 

set wait  = 0
set host  = "localhost"
set team  = "LiraTeam" 
set dir   = "src"
set prog  = "${dir}/trilearn_player"
set coach = "${dir}/trilearn_coach"
set pconf = "${dir}/player.conf"
set fconf = "${dir}/formations.conf"

echo "*****************************************************************"
echo "* UvA_Trilearn 2003 - University of Amsterdam, The Netherlands  *"
echo "* Base code version                                             *"
echo "* Created by:           Jelle Kok                               *"
echo "* Research Coordinator: Nikos Vlassis                           *" 
echo "* Team Coordinator:     Frans Groen                             *"
echo "* Copyright 2000-2001.  Jelle Kok and Remco de Boer             *"
echo "* Copyright 2001-2002.  Jelle Kok                               *"
echo "* Copyright 2002-2003.  Jelle Kok                               *"
echo "* All rights reserved.                                          *"
echo "*****************************************************************"


echo "*****************************************************************"
echo "* MDPtoolbox: Markov Decision Processes Toolbox                 *"
echo "* Copyright (C) 2009  INRA                                      *"
echo "* Redistribution and use in source and binary forms, with or    *"
echo "* without modification, are permitted provided that the         *" 
echo "* following conditions are met:                                 *"
echo "*   -> Redistributions of source code must retain the above     *"
echo "*      copyright notice, this list of conditions and the        *"
echo "*      following disclaimer.                                    *"
echo "*   -> Redistributions in binary form must reproduce the above  *"
echo "*      copyright notice, this list of conditions and the        *"
echo "*      following disclaimer in the documentation and/or other   *"
echo "*      materials provided with the distribution.                *"
echo "*   -> Neither the name of the <ORGANIZATION> nor the names of  *"
echo "*      its contributors may be used to endorse or promote       *" 
echo "*       products derived from this software without specific    *"
echo "*       prior written permission.                               *"
echo "* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND        *"
echo "* CONTRIBUTORS ´AS IS´ AND ANY EXPRESS OR IMPLIED WARRANTIES,   *"
echo "* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF      *"
echo "* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE      *"
echo "* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR         *"
echo "* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  *"
echo "* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,      *"
echo "* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR        *"
echo "* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS          *"
echo "* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,  *"
echo "* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     *"
echo "* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF *"
echo "* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH     *"
echo "* DAMAGE.                                                       *"
echo "*****************************************************************"

#first check if the last two supplied arguments are no numbers and represent
#<host-name> or <host-name> <team-name>
if( $#argv > 0 && ($argv[$#argv] !~ [0123456789]* || $argv[$#argv] =~ *.* ) ) then
  @ second_last = $#argv - 1  
  if( $#argv > 1 && ($argv[$second_last] !~ [0123456789]* || $argv[$second_last] =~ *.* ) ) then
      set host = $argv[$second_last]
      set team = $argv[$#argv]
  else
      set host = $argv[$#argv]
  endif
endif

#then if first argument is a number, start only the players with the numbers
#as supplied on the prompt, otherwise start all players.
if( $1 =~ [0123456789]* && $1 !~ *.* ) then
  echo "$argv[$#argv]"
    echo "$1"
  foreach arg ($argv)
    if( $arg =~ [123456789]* && $arg !~ *.*) then
      ${prog} -num ${arg} -host ${host} -team ${team} -f ${fconf} -c ${pconf} &
      sleep $wait
    else if( $arg =~ [0]* ) then
      sleep 2
      ${coach} -host ${host} -team ${team} -f ${fconf} &
    endif
  end
else
  set i = 1
  while ( ${i} <12 )
    ${prog} -number ${i} -host ${host} -team ${team}  -f ${fconf} -c ${pconf} &
    sleep $wait
    @ i++
  end
  sleep 2
  ${coach} -host ${host} -team ${team} -f ${fconf}  &
endif

