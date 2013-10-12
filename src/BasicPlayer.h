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

/*! \file BasicPlayer.h
<pre>
<b>File:</b>          BasicPlayer.h
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       10/12/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
               BasicPlayer. The BasicPlayer is the class where the
               available skills for the agent are defined.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
10/12/2000       Jelle Kok       Initial version created
</pre>
*/

#ifndef _BASICPLAYER_
#define _BASICPLAYER_


/******************************************************************************/
/*****************Definicoes para a classe Matrizes****************************/
 //numero de estados
#define numEstados 82944
 //numero de acoes
#define numAcoes 13 
 //numero de chaves
#define inum 9 
/******************************************************************************/

#include "ActHandler.h"
#include <stdio.h>

#include <stdlib.h>
#include <time.h>
#include <string.h>





/******************************************************************************/
/******************** CLASS APRENDIZAGEM_REFORCO ******************************/
/******************************************************************************/
/* Esta classe possui as principais funções e variáveis para executar o algoritmo
 * Q_Learning.*/

class Aprendizagem_Reforco
{
	public:
	Matrizes *MA ;
	WorldModel *WM ;

	int Partida_cont, player_Num;
	int Iteracao_cont;
	int saldo, saldo_atual, verInf, gols_adv, gols_nosso;
    double desconto;
	int s, s_new, acao;
	int cont_a[10][13];
	int i, j, p;
	int ciclo_atual, ciclo_ant ;

	char relatorio[19];
	char resultados[16];

	AngDeg dir_bola ;
	AngDeg dir_bola2 ;

	double vel_bola ;
	double vel_bola2 ;

	Aprendizagem_Reforco                      (                               );
	~Aprendizagem_Reforco                     (                               );
	void SalvaInformacoes                     (                               );
	void SalvaResultados                      (                               );
	void InicializaInformacoes                (                               );
	void AtualizaSaldoGol                     (                               );
	void AtualizaPlayerNum                    ( int num                       );
	int CalculaReforco                        ( int action, int So, int Sf    );
	void AdicionaContA                        ( int action                    );

	struct noh
	{
		long CA_estadoAcao;
		long CA_estadoFinal;
		int CA_acao;
		noh* pointer;
	};
	typedef noh* noChain;

	int CA_numActions;
	int CA_possession;
	noChain chainAction[10];
	noh* ultimoNo[10];

	// Inicia a cadeia de acao
	void initChain                           ( noh* chain[], int chain_lenght );
	// Adiciona um novo nó ao fim da cadeia
	void addNo_Ultimo						 ( noh* chain[], int index        );
	// Metodo usado para adicao de nós
	noh* addNo								 ( noh* newNo                     );
	// Metodo que procura ultimo nó
	noh* ultNo								 ( noh* lookNo                    );
	// Metodo que deleta a cadeia de acoes
	void delCA                               ( noh* chain[], int index        );
	// Deleta o ultimo elemento da cadeia
	void delNoUlt							 ( noh* chain[], int index        );
	// Atualiza o ultimo No do respectivo jogador (index), na cadeia enviada
	void atualiza_ultNo                      ( noh* chain[], int index        );
};	

extern Logger Log; /*!< This is a reference to Logger to write log info to*/

/*! This class defines the skills that can be used by an agent. No
    functionality is available that chooses when to execute which skill, this
    is done in the Player class. The WorldModel is used to determine the way
    in which the skills are performed. */
class BasicPlayer
{
protected:
  ActHandler      *ACT; /*!< ActHandler to which commands can be sent        */
  WorldModel      *WM;  /*!< WorldModel that contains information of world   */
  Matrizes        *MA;
  Aprendizagem_Reforco *AR;
  ServerSettings  *SS;  /*!< All parameters used by the server               */
  PlayerSettings  *PS;  /*!< All parameters used for the player              */
 
  double dPlayerVersion;				/*the version of the player. Some new functions changes their
	 functionality according to the player version. Example: Tackle*/
  ////////////////////////// LOW-LEVEL SKILLS /////////////////////////////////

  SoccerCommand   alignNeckWithBody       (                                  );
  SoccerCommand   turnBodyToPoint         ( VecPosition   pos,
                                            int           iPos = 1           );
  SoccerCommand   turnBackToPoint         ( VecPosition   pos,
                                            int           iPos = 1           );
  SoccerCommand   turnSideToPoint( VecPosition pos, AngDeg angle, int iCycles  =1);
  SoccerCommand   turnNeckToPoint         ( VecPosition   pos,
                                            SoccerCommand com                );
  SoccerCommand   searchBall              (                                  );
  SoccerCommand   dashToPoint             ( VecPosition   pos,
                                            int           iCycles = 1,        
										   bool 		sideDash = false);
  SoccerCommand   freezeBall              (                                  );
  SoccerCommand   kickBallCloseToBody     ( AngDeg        ang,
					    double        dKickRatio = 0.16  );
  SoccerCommand   accelerateBallToVelocity( VecPosition   vel                );
  SoccerCommand   catchBall               (                                  );
  SoccerCommand   communicate             ( char          *str               );
  SoccerCommand   teleportToPos           ( VecPosition   pos                );
  SoccerCommand   listenTo                ( ObjectT       obj                );
  SoccerCommand   tackle                  ( double angle, bool foul              );

  ////////////////////////// INTERMEDIATE SKILLS //////////////////////////////

  SoccerCommand   turnBodyToObject        ( ObjectT       o                  );
  SoccerCommand   turnNeckToObject        ( ObjectT       o,
                                            SoccerCommand com                );
  SoccerCommand   directTowards           ( VecPosition   posTo,
                                            AngDeg        angWhenToTurn,
                                            VecPosition   *pos = NULL,
                                            VecPosition   *vel = NULL,
                                            AngDeg        *angBody  = NULL   );
  SoccerCommand   moveToPos               ( VecPosition   posTo,
                                            AngDeg        angWhenToTurn,
                                            double        dDistDashBack = 0.0,
                                            bool          bMoveBack = false,
                                            int           iCycles = 1        );
  SoccerCommand moveToPos2( VecPosition posTo, AngDeg angWhenToTurn,
                            double angle);
  SoccerCommand   collideWithBall         (                                  );
  SoccerCommand   interceptClose          (                                  );
  SoccerCommand   interceptCloseGoalie    (                                  );
  SoccerCommand   kickTo                  ( VecPosition   posTarget,
                                            double        dEndSpeed          );
  SoccerCommand   turnWithBallTo          ( AngDeg        ang,
                                            AngDeg        angKickThr,
                                            double        dFreezeThr         );
  SoccerCommand   moveToPosAlongLine      ( VecPosition   pos,
                                            AngDeg        ang,
                                            double        dDistThr,
                                            int           iSign,
                                            AngDeg        angThr,
                                            AngDeg        angCorr            );


  ////////////////////////// HIGH-LEVEL SKILLS ////////////////////////////////

  SoccerCommand   intercept               ( bool          isGoalie           );
  SoccerCommand   dribble                 ( AngDeg        ang,
                                            DribbleT      d                  );

  SoccerCommand   AR_SafeDribblePlus30    (                                  );
  SoccerCommand   AR_SafeDribbleMinus30   (                                  );
  SoccerCommand   AR_SafeDribblePlus90    (                                  );
  SoccerCommand   AR_SafeDribbleMinus90   (                                  );
  SoccerCommand   AR_SafeDribble          (                                  );
  SoccerCommand   AR_FastDribble          (                                  );

	
  SoccerCommand   directPass              ( VecPosition   pos,
                                            PassT         passType           );
	//AR_change
  SoccerCommand   AR_directPassNormal     (                                  );
  SoccerCommand   AR_directShootGoal      (                                  );

	
  SoccerCommand   leadingPass             ( ObjectT       o,
                                            double        dDist,
                                            DirectionT    dir = DIR_NORTH    );
  SoccerCommand   throughPass             ( ObjectT       o,
                                            VecPosition   posEnd,
                                            AngDeg        *angMax = NULL     );
  SoccerCommand   outplayOpponent         ( ObjectT       o,
                                            VecPosition   pos,
                                            VecPosition   *posTo = NULL      );
  SoccerCommand   clearBall               ( ClearBallT    type,
                                            SideT         s = SIDE_ILLEGAL,
                                            AngDeg        *angMax = NULL     );
  SoccerCommand   mark                    ( ObjectT       o,
                                            double        dDist,
                                            MarkT         mark               );
  SoccerCommand   AR_markOppAndBall       (                                  );
  SoccerCommand   AR_markOppAndGoal       (                                  );

  SoccerCommand   AR_moveToStrategicPos   (                                  );
	
  SoccerCommand   defendGoalLine          ( double        dDist              );
  SoccerCommand   interceptScoringAttempt (                                  );
  SoccerCommand   holdBall                (                                  );

  ////////////////////////// UTILITY METHODS //////////////////////////////////

  VecPosition     getThroughPassShootingPoint( ObjectT       objTeam,
                                               VecPosition   posEnd,
					       AngDeg        *angMax         );
  VecPosition     getInterceptionPointBall(    int           *iCyclesBall,
                                               bool          isGoalie        );
  VecPosition     getActiveInterceptionPointBall
                                          ( int           *iCyclesBall,
                                            bool          isGoalie           );
  VecPosition     getDribblePoint         ( DribbleT      dribble,
                                            double        *dDist             );
  VecPosition     getShootPositionOnLine  ( VecPosition   p1,
                                            VecPosition   p2,
                                            AngDeg        *angLargest = NULL );
  double          getEndSpeedForPass      ( ObjectT       o,
                                            VecPosition   posPass            );
  VecPosition     getMarkingPosition      ( ObjectT       o,
                                            double        dDist,
                                            MarkT         mark               );
  double getPlayerVersion();
  SoccerCommand syncSee();
} ;



#endif
