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

/*! \file PlayerTeams.cpp
<pre>
<b>File:</b>          PlayerTest.cpp
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       10/12/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class definitions for the
                      Player that are used to test the teams' high level
                      strategy.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
10/12/2000        Jelle Kok       Initial version created
</pre>
 */

#include "Player.h"
#include "BasicPlayer.h"

/*!This method is the first complete simple team and defines the actions taken
   by all the players on the field (excluding the goalie). It is based on the
   high-level actions taken by the simple team FC Portugal that it released in
   2000. The players do the following:
   - if ball is kickable
       kick ball to goal (random corner of goal)
   - else if i am fastest player to ball 
       intercept the ball
   - else
       move to strategic position based on your home position and pos ball */
SoccerCommand Player::deMeer5(  )
{


	/***************************** Inicializacoes *********************************/
	MA->AtualizaNumero(WM->getPlayerNumber());
	//cerr << WM->getPlayerNumber() << ": Carregando matriz estados\n";
	MA->InicializaMatrizEstados();
	//cerr << WM->getPlayerNumber() << ": Carregando matriz R\n";
	MA->InicializaMatrizR();
	//cerr << WM->getPlayerNumber() << ": Carregando matriz Q\n";
	MA->InicializaMatrizQ();
	//cerr << WM->getPlayerNumber() << ": Carregando matriz T\n";
	MA->InicializaMatrizT();
	//cerr << WM->getPlayerNumber() << ": Todas matrizes iniciadas\n";

	MA->AtualizaNumJogo( AR->Partida_cont ) ;
	SoccerCommand soc(CMD_ILLEGAL);
	VecPosition   posAgent = WM->getAgentGlobalPosition();
	VecPosition   posBall  = WM->getBallPos();
	int           iTmp;
	int e[inum], e_new[inum];
	int estado[inum];
	int player;
	int cont_a, reforco, r, i, j, p;
	double prob, delta, dQ, desconto, maxQ, coef1, coef2;
	desconto = 0.9;
	AR->ciclo_atual = WM->getCurrentCycle() ;
	int possession;

	if( WM->isBallInOurPossesion( )==true)
		possession = 1;
	else possession = 0;

	player = WM->getPlayerNumber();
		p = player - 2 ;
	bool kickable = WM->isBallKickable();

	// Construção do vetor de estado
	estado[0] = WM->AR_getAgentPos();
	estado[1] = WM->AR_getBallPos();
	estado[2] = possession;
	estado[3] = WM->AR_getBallRelDistance( player, kickable );
	estado[4] = WM->AR_getCloTeammateRelDist();
	estado[5] = WM->AR_getCloOpponentRelDist();
	estado[6] = WM->AR_getRelativeOppAng();
	estado[7] = WM->AR_getAgentGloBodyAngle();
	estado[8] = WM->AR_getAngleChavei5();

	/********************* Básico para iniciar o jogo *****************************/
	if( WM->isBeforeKickOff( ) )
	{
		AR->CA_numActions = 0;
		MA->InsereVetorA( 9, 0 );
		if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
				posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )
		{
			formations->setFormation( FT_INITIAL );       // go to kick_off formation
			ACT->putCommandInQueue(soc=teleportToPos( WM->getStrategicPosition() ));
		}
		if( WM->isKickOffUs( ) && WM->getPlayerNumber() == 10 ) // 10 takes kick
		{
			AR->CA_possession = 1; // Agente começa com a bola
			if( kickable )
			{
				soc = AR_directPassNormal() ;
				Log.log( 100, "take kick off" );
			}
			else
			{
				soc = intercept( false );
				Log.log( 100, "move to ball to take kick-off" );
			}
			ACT->putCommandInQueue( soc );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			return soc;
		}
	}
	else if( ( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true ) && WM->isBallInOwnPenaltyArea() == true )
	{
		if(WM->getPlayerNumber() == 10)
			AR->CA_possession = 0; // Eles começam com a bola

		MA->InsereVetorA( 9, 0 );
		soc = moveToPos(WM->getStrategicPosition(),
				PS->getPlayerWhenToTurnAngle());
		ACT->putCommandInQueue( soc );
	}

	else if( AR->ciclo_atual != AR->ciclo_ant )
	{

		/************************** Atualiza Saldo de Gol *****************************/

		AR->saldo_atual = AR->saldo ;
		AR->saldo = WM->getGoalDiff( ) ;
		AR->dir_bola = WM->getBallDirection() ;
		AR->vel_bola = WM->getBallSpeed() ;

		if( AR->saldo != AR->saldo_atual )
		{
			if( AR->saldo < AR->saldo_atual ) AR->gols_adv++ ;
			else AR->gols_nosso++ ;
		}

		/******************************* Q-Learning ***********************************/
		formations->setFormation( FT_343_ATTACKING );

		for( i=0; i < inum; i++ )
		{
			MA->InsereVetorS( i, estado[i] ) ;
		}

		AR->AtualizaPlayerNum( player ) ;
		AR->InicializaInformacoes();

		/******************************** Iteração ************************************/
		if( MA->verAcao == 5 )
		{
			MA->verAcao = 0 ;

			//Atualiza novo Estado
			AR->s_new = MA->RetornaEstado();

			if( AR->ciclo_atual > 50 )
			{
				int actNum = 1; // Numero crescente da acao, comencando por 1 na ultima (primeira acao tomada na cadeia vai ter actNum maior)
				AR->ultNo( AR->chainAction[p] )->CA_estadoFinal = AR->s_new; // p = player - 2 (referente a posicao no vetor. Ex: Num. 2 -> p = 0 (min); Num. 11 -> p = 9 (max)
				if( possession != AR->CA_possession && AR->CA_numActions > 1)
				{
					if( AR->CA_possession == 1 ) // Perdemos a bola
					{
						AR->CA_possession = 0;
						reforco = 27*log(actNum) - 15; // Colocando fora do looping é possivel conceder um reforco diferenciado a ultima acao

						AR->atualiza_ultNo( AR->chainAction, player );

						MA->InsereReforco( AR->ultimoNo[p]->CA_acao,
								AR->ultimoNo[p]->CA_estadoAcao, reforco);

						Atualiza_MatrizQ( reforco, AR->ultimoNo[p]->CA_estadoFinal,
								AR->ultimoNo[p]->CA_estadoAcao,
								AR->ultimoNo[p]->CA_acao );

						AR->delNoUlt( AR->chainAction, player);

						while( AR->ultNo( AR->chainAction[player] )->pointer != NULL )
						{
							AR->atualiza_ultNo( AR->chainAction, player ); // Apos deletar o ultimo nó é preciso atualizar o vetor dos ultimos nós

							actNum++;
							reforco = 27*log(actNum)-15;

							MA->InsereReforco( AR->ultimoNo[p]->CA_acao,
									AR->ultimoNo[p]->CA_estadoAcao, reforco);

							Atualiza_MatrizQ( reforco, AR->ultimoNo[p]->CA_estadoFinal,
									AR->ultimoNo[p]->CA_estadoAcao,
									AR->ultimoNo[p]->CA_acao );

							AR->delNoUlt( AR->chainAction, player);
						}
						AR->delCA(AR->chainAction, p);
						/*
				  if( WM->isFreeKickThem() ) // Tiro livre pra eles
				  {



				  while(!AR->chainAction[player]->pointer == NULL)
				  {

				  }
				  MA->InsereReforco( AR->acao, AR->s, reforco);
				  }// Apos deletar o ultimo no é preciso atualizar o vetor dos ultimos nós
				  else if( WM->isGoalKickThem() ) // Tiro de meta pra eles
				  {

				  }
				  else if( WM->isCornerKickThem() ) // Escanteio pra eles
				  {

				  }
				  else if( WM->isOffsideUs() ) // Ficamos em impedimento
				  {

				  }
				  else if( WM->isKickInThem() ) // Lateral pra eles
				  {

				  }
				  else if( WM->isFreeKickFaultUs() ) // Cometemos uma irregularidade apos um tiro livre
				  {

				  }
				  else if( WM->isYellowCardUs() ) // Tomamos cartao amarelo
				  {

				  }
				  else if( WM->isRedCardUs() ) // Tomamos cartao vermelho
				  {

				  }
				  else if( WM->isBackPassUs() ) // Recuamos para o goleiro e ele pegou
				  {

				  }
				  else if( WM->isPenaltyThem() ) // Penalte para eles
				  {

				  }*/
					}
					else // bola recuperada
					{
						AR->CA_possession = 1;
						reforco = 2*exp(actNum/2) + 20;

						AR->atualiza_ultNo( AR->chainAction, player );

						MA->InsereReforco( AR->ultimoNo[p]->CA_acao,
								AR->ultimoNo[p]->CA_estadoAcao, reforco);
						Atualiza_MatrizQ( reforco, AR->ultimoNo[p]->CA_estadoFinal,
								AR->ultimoNo[p]->CA_estadoAcao,
								AR->ultimoNo[p]->CA_acao );

						AR->delNoUlt( AR->chainAction, player);

						while( AR->ultNo( AR->chainAction[player] )->pointer != NULL )
						{
							AR->atualiza_ultNo( AR->chainAction, player );

							actNum++;
							reforco = 2*exp(actNum)+20;

							MA->InsereReforco( AR->ultNo( AR->chainAction[player] )->CA_acao,
									AR->ultNo( AR->chainAction[player] )->CA_estadoAcao, reforco);

							Atualiza_MatrizQ( reforco, AR->ultNo( AR->chainAction[player] )->CA_estadoFinal,
									AR->ultNo( AR->chainAction[player] )->CA_estadoAcao,
									AR->ultNo( AR->chainAction[player] )->CA_acao );

							AR->delNoUlt( AR->chainAction, player);
						}
					}
					/*else if( AR->saldo > AR->saldo_atual ) // Fizemos um gol
			  {
				  AR->CA_possession = 0;
				  reforco = 2*log(actNum/2)+20;
				  MA->InsereReforco( AR->ultNo( AR->chainAction[player] )->CA_acao,
						  AR->ultNo( AR->chainAction[player] )->CA_estadoAcao, reforco);

				  Player::Atualiza_MatrizQ( reforco, AR->ultNo( AR->chainAction[player] )->CA_estadoFinal,
						  AR->ultNo( AR->chainAction[player] )->CA_estadoAcao,
						  AR->ultNo( AR->chainAction[player] )->CA_acao );

				  AR->delNoUlt( AR->chainAction, player);

				  while( AR->ultNo( AR->chainAction[player] )->pointer != NULL )
				  {
					  actNum++;
					  reforco = 2*exp(actNum)+20;
					  MA->InsereReforco( AR->ultNo( AR->chainAction[player] )->CA_acao,
							  AR->ultNo( AR->chainAction[player] )->CA_estadoAcao, reforco);

					  player::Atualiza_MatrizQ( reforco, AR->ultNo( AR->chainAction[player] )->CA_estadoFinal,
							  AR->ultNo( AR->chainAction[player] )->CA_estadoAcao,
							  AR->ultNo( AR->chainAction[player] )->CA_acao );
					  }
				  }*/

				}

				AR->CA_numActions = 0;
			}

			/***************************** Calculo Reforço ********************************/

			for( i = 0; i < inum; i++ )
			{
				e[i] = MA->RetornachaveEstado( AR->s, i ) ;
				e_new[i] = MA->RetornachaveEstado( AR->s_new, i ) ;
			}

			//if( WM->getPlayerNumber() == 8 )cerr << AR->ciclo_atual << " " << e[3] << e_new[3] << endl;
			//Reforco do Gol tomado ou não
			if( AR->saldo != AR->saldo_atual )
			{
				if( AR->saldo < AR->saldo_atual ) reforco += -100 ;
				else reforco += 75 ;
			}

			//reforco da Posição do jogador
			if( player == 2 || player == 3 || player == 4)
			{
				if( e_new[0] == 0 || e_new[0] == 1 || e_new[0] == 2 || e_new[0] == 3
						|| e_new[0] == 4 || e_new[0] == 5 )
					reforco += 5;
				else reforco += -20;
			}

			if( player == 11 || player == 10 || player == 9)
			{
				if( e_new[0] == 6 || e_new[0] == 7 || e_new[0] == 8 || e_new[0] == 9
						|| e_new[0] == 10 || e_new[0] == 11 )
					reforco += 5;
				else reforco += -5;
			}

			//reforço da posição da bola
			if( e_new[1] == 0 ) reforco += -15;
			else if( e_new[1] == 2 ) reforco += 20;

			//reforço da posse de bola
			if( e_new[2] == 1 ) reforco += 5;
			else reforco += -10;

			//reforço da Distancia da bola
			if( e[3] > 0 && e_new[3] == 0 ) reforco += 10;
			if( e[3] == 0 && e_new[3] > 0 ) reforco += -20;

			//reforço da distancia ao aliado e oponente mais proximo
			if( e_new[4] == 0 || e_new[5] == 0 ) reforco += -5;

			//reforço do angulo ao oponente
			if( e_new[6] == 3 ) reforco += 5;

			//reforço do angulo do agente
			if( e_new[7] == 2 ) reforco += -5;
			else reforco += 5;


			MA->InsereReforco( AR->acao, AR->s, reforco);


			/*************************** Atualiza Matriz Q ********************************/

			r = MA->RetornaReforco( AR->acao, AR->s );


			delta = r + desconto*MA->RetornaMaxQ(AR->s_new) ;
			//Coeficiente de decaimento ajustado para
			// 1/((ln iteracao)/10000)
			coef1 = ((1/(log((AR->Iteracao_cont/24000.0) + 2.72))));
			coef2 = (-1)*WM->getGoalDiff( )/5.0 ;
			if( coef2 < 0.0 ) coef2 = 0.0 ;
			//if( coef1 < coef2 ) coef1 = coef2 ;
			coef1 = coef1 + coef2 ;
			if( coef1 > 1 ) coef1 = 1 ;
			dQ = coef1*delta ;

			MA->InsereMatrizQ( AR->acao, AR->s, (1-coef1)*MA->RetornaValorQ(AR->acao, AR->s) + dQ ) ;
			// (1-coef1)*Q(s,a) + coef1*(reforco + desconto*maxQ(s:))
			MA->InsereTransicao( AR->acao, AR->s, AR->s_new ) ;
			AR->Iteracao_cont++ ;

			/******************************************************************************/

			AR->dir_bola2 = AR->dir_bola ;
			AR->vel_bola2 = AR->vel_bola ;
		}




		/****************************** Escolha Ação **********************************/


		MA->verAcao = 5 ;

		AR->ultNo( AR->chainAction[p] )->CA_estadoFinal = AR->s;
		AR->s = AR->s_new;

		AR->addNo_Ultimo( AR->chainAction, player );

		srand(WM->getPlayerNumber()*rand());
		prob = (rand()%1000001)/1000000.00;

		bool randAction = false;

		if( prob < (1.00 - (1.00/log((AR->Iteracao_cont/24000) + 2.72 ))))
		{
			//cerr << "Acao Otima";
			AR->acao = MA->RetornaAcaoOtima(AR->s, estado[3]) ;
			//cerr << acao << '\n';
			//if( WM->getPlayerNumber() == 9 ) cerr << "Soh tah entrando aqui" << AR->acao << '\n';
			/*if((AR->acao >= 0 && AR->acao <= 8) && estado[3] != 0)
	  		{
	  			cerr << "Tentei chutar mas nao consegui\n";
	  			randAction = true;
	  		}*/
		}
		else
			randAction = true;
		soc = intercept (false);
		MA->InsereVetorA( 9, 5 );

		if(randAction == true)
		{
			//if( WM->getPlayerNumber() == 9 ) cerr << "Tah aqui tbm" << AR->acao << '\n';
			//cerr << "Acao Aleatoria";

			if( estado[3] == 0 )
			{
				AR->acao = rand()%9;
			}
			else
			{
				prob = rand()%4;
				if( prob == 0 ) AR->acao = 9;
				else if( prob == 1 ) AR->acao = 10;
				else if( prob == 2 ) AR->acao = 11;
				else if( prob == 3 ) AR->acao = 12;
			}
		}

		AR->AdicionaContA( AR->acao ) ;
		WM->acaoWM == AR->acao;

		AR->ultNo(AR->chainAction[p])->CA_acao = AR->acao;
		AR->ultNo(AR->chainAction[p])->CA_estadoAcao = AR->s_new;
		AR->ultNo(AR->chainAction[p])->CA_estadoFinal = -1;
		AR->CA_numActions++;

		/************************** Conversão em Comando ******************************/

		int infor = AR->acao;
		switch( AR->acao )
		{
		case 0:
			soc = AR_SafeDribble ();
			break;
		case 1:
			soc = AR_FastDribble ();
			break;
		case 2:
			soc = AR_SafeDribblePlus30 ();
			break;
		case 3:
			soc = AR_SafeDribbleMinus30 ();
			break;
		case 4:
			soc = AR_SafeDribblePlus90 ();
			break;
		case 5:
			soc = AR_SafeDribbleMinus90 ();
			break;
		case 6:
			soc = AR_directPassNormal ();
			break;
		case 7:
			soc = AR_directShootGoal ();
			break;
		case 8:
			soc = freezeBall( );
			break;
		case 9:
			soc = intercept (false);
			MA->InsereVetorA( 9, 5 );
			break;
		case 10:
			soc = AR_markOppAndBall ();
			//if( soc == CMD_ILLEGAL ) soc = intercept (false);
			break;
		case 11:
			soc = turnBodyToObject( OBJECT_BALL ) ;//AR_markOppAndGoal ();
			//if( soc == CMD_ILLEGAL ) soc = AR_moveToStrategicPos ();
			break;
		case 12:
			soc = moveToPos(WM->getStrategicPosition(),
					PS->getPlayerWhenToTurnAngle());
			break;
			//Ação Padrão é interceptar a bola se estiver sem a bola e
			//isolar a bola se estiver com ela.
		default:
			if( estado[3] == 0 ) soc = clearBall( CLEAR_BALL_DEFENSIVE );
			else soc = intercept (false);
			break;
		}


		/********************* Inclusão da ação na fila de execução *******************/

		if( AR->acao >= 0 && AR->acao < 9 && !kickable )
			printf("Impossivel executar chute!\n");

		//Análise da stamina à parte da Aprendizagem e proteção contra ações
		// indevidas como chutar a bola sem ter a posse.
		if( soc.commandType == CMD_DASH &&             // if stamina low
				WM->getAgentStamina().getStamina() <
				SS->getRecoverDecThr()*SS->getStaminaMax()+200 )
		{
			soc.dPower = 30.0 * WM->getAgentStamina().getRecovery(); // dash slow
			if( estado[3] == 0 )
			{
				if( AR->acao >= 0 && AR->acao <= 8 )
					ACT->putCommandInQueue( soc );
			}
			else if( AR->acao >= 9 && AR->acao <= 12 )
				ACT->putCommandInQueue( soc );

		}
		else                                           // if stamina high
		{
			if( estado[3] == 0 )
			{
				if( AR->acao >= 0 && AR->acao <= 8 )
					ACT->putCommandInQueue( soc );
			}
			else if( AR->acao >= 9 && AR->acao <= 12 )
				ACT->putCommandInQueue( soc );
			// dash as intended
		}

		AR->ciclo_ant = AR->ciclo_atual ;
	}

	return soc;
}	



/*!This method is a simple goalie based on the goalie of the simple Team of
   FC Portugal. It defines a rectangle in its penalty area and moves to the
   position on this rectangle where the ball intersects if you make a line
   between the ball position and the center of the goal. If the ball can
   be intercepted in the own penalty area the ball is intercepted and catched.
 */
SoccerCommand Player::deMeer5_goalie(  )
{
	int i;

	SoccerCommand soc;
	VecPosition   posAgent = WM->getAgentGlobalPosition();
	AngDeg        angBody  = WM->getAgentGlobalBodyAngle();

	// define the top and bottom position of a rectangle in which keeper moves
	static const VecPosition posLeftTop( -PITCH_LENGTH/2.0 +
			0.7*PENALTY_AREA_LENGTH, -PENALTY_AREA_WIDTH/4.0 );
	static const VecPosition posRightTop( -PITCH_LENGTH/2.0 +
			0.7*PENALTY_AREA_LENGTH, +PENALTY_AREA_WIDTH/4.0 );

	// define the borders of this rectangle using the two points.
	static Line  lineFront = Line::makeLineFromTwoPoints(posLeftTop,posRightTop);
	static Line  lineLeft  = Line::makeLineFromTwoPoints(
			VecPosition( -50.0, posLeftTop.getY()), posLeftTop );
	static Line  lineRight = Line::makeLineFromTwoPoints(
			VecPosition( -50.0, posRightTop.getY()),posRightTop );


	if( WM->isBeforeKickOff( ) )
	{
		if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
				posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )
		{
			formations->setFormation( FT_INITIAL );       // go to kick_off formation
			ACT->putCommandInQueue( soc=teleportToPos(WM->getStrategicPosition()) );
		}
		else                                            // else turn to center
		{
			ACT->putCommandInQueue( soc = turnBodyToPoint( VecPosition( 0, 0 ), 0 ));
			ACT->putCommandInQueue( alignNeckWithBody( ) );
		}
		return soc;
	}

	if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
	{                                                // confidence ball too  low
		ACT->putCommandInQueue( searchBall() );        // search ball
		ACT->putCommandInQueue( alignNeckWithBody( ) );
	}
	else if( WM->getPlayMode() == PM_PLAY_ON || WM->isFreeKickThem() ||
			WM->isCornerKickThem() )
	{
		if( WM->isBallCatchable() )
		{
			ACT->putCommandInQueue( soc = catchBall() );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
		}
		else if( WM->isBallKickable() )
		{
			soc = kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );
			ACT->putCommandInQueue( soc );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
		}
		else if( WM->isInOwnPenaltyArea( getInterceptionPointBall( &i, true ) ) &&
				WM->getFastestInSetTo( OBJECT_SET_PLAYERS, OBJECT_BALL, &i ) ==
						WM->getAgentObjectType() )
		{
			ACT->putCommandInQueue( soc = intercept( true ) );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
		}
		else
		{
			// make line between own goal and the ball
			VecPosition posMyGoal = ( WM->getSide() == SIDE_LEFT )
            		 ? SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_L, SIDE_LEFT )
			: SoccerTypes::getGlobalPositionFlag(OBJECT_GOAL_R, SIDE_RIGHT);
			Line lineBall = Line::makeLineFromTwoPoints( WM->getBallPos(),posMyGoal);

			// determine where your front line intersects with the line from ball
			VecPosition posIntersect = lineFront.getIntersection( lineBall );

			// outside rectangle, use line at side to get intersection
			if (posIntersect.isRightOf( posRightTop ) )
				posIntersect = lineRight.getIntersection( lineBall );
			else if (posIntersect.isLeftOf( posLeftTop )  )
				posIntersect = lineLeft.getIntersection( lineBall );

			if( posIntersect.getX() < -49.0 )
				posIntersect.setX( -49.0 );

			// and move to this position
			if( posIntersect.getDistanceTo( WM->getAgentGlobalPosition() ) > 0.5 )
			{
				soc = moveToPos( posIntersect, PS->getPlayerWhenToTurnAngle() );
				ACT->putCommandInQueue( soc );
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			}
			else
			{
				ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			}
		}
	}
	else if( WM->isFreeKickUs() == true || WM->isGoalKickUs() == true )
	{
		if( WM->isBallKickable() )
		{
			if( WM->getTimeSinceLastCatch() == 25 && WM->isFreeKickUs() )
			{
				// move to position with lesser opponents.
				if( WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS,
						Circle(posRightTop, 15.0 )) <
						WM->getNrInSetInCircle( OBJECT_SET_OPPONENTS,
								Circle(posLeftTop,  15.0 )) )
					soc.makeCommand( CMD_MOVE,posRightTop.getX(),posRightTop.getY(),0.0);
				else
					soc.makeCommand( CMD_MOVE,posLeftTop.getX(), posLeftTop.getY(), 0.0);
				ACT->putCommandInQueue( soc );
			}
			else if( WM->getTimeSinceLastCatch() > 28 )
			{
				soc = clearBall(CLEAR_BALL_DEFENSIVE) ; //kickTo( VecPosition(0,posAgent.getY()*2.0), 2.0 );
				ACT->putCommandInQueue( soc );
			}
			else if( WM->getTimeSinceLastCatch() < 25 )
			{
				VecPosition posSide( 0.0, posAgent.getY() );
				if( fabs( (posSide - posAgent).getDirection() - angBody) > 10 )
				{
					soc = turnBodyToPoint( posSide );
					ACT->putCommandInQueue( soc );
				}
				ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
			}
		}
		else if( WM->isGoalKickUs()  )
		{
			ACT->putCommandInQueue( soc = intercept( true ) );
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
		}
		else
			ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
	}
	else
	{
		ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
		ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
	}
	return soc;
}


/*  if( WM->isKickOffUs( ) && WM->getPlayerNumber() == 9 ) // 9 takes kick
    {
      if( WM->isBallKickable() )
      {

	    soc = AR_directPassNormal() ;
        Log.log( 100, "take kick off" );        
      }
      else
      {
        soc = intercept( false );
        Log.log( 100, "move to ball to take kick-off" );
      }  
      ACT->putCommandInQueue( soc );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      return soc;
    }  
    if( formations->getFormation() != FT_INITIAL || // not in kickoff formation
        posAgent.getDistanceTo( WM->getStrategicPosition() ) > 2.0 )  
    {
      formations->setFormation( FT_INITIAL );       // go to kick_off formation
      ACT->putCommandInQueue( soc=teleportToPos( WM->getStrategicPosition() ));
    }
    else                                            // else turn to center
    {
      ACT->putCommandInQueue( soc=turnBodyToPoint( VecPosition( 0, 0 ), 0 ) );
      ACT->putCommandInQueue( alignNeckWithBody( ) );
    }
  }
  else
  {
    formations->setFormation( FT_433_OFFENSIVE );
    soc.commandType = CMD_ILLEGAL;
    if( WM->getConfidence( OBJECT_BALL ) < PS->getBallConfThr() )
    {
      ACT->putCommandInQueue( soc = searchBall() );   // if ball pos unknown
      ACT->putCommandInQueue( alignNeckWithBody( ) ); // search for it
    }
    else if( WM->isBallKickable())                    // if kickable
    {


	  soc = AR_directPassNormal() ;


      ACT->putCommandInQueue( soc );
      ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      Log.log( 100, "kick ball" );
    }
    else if( WM->getFastestInSetTo( OBJECT_SET_TEAMMATES, OBJECT_BALL, &iTmp )
              == WM->getAgentObjectType()  && !WM->isDeadBallThem() )
    {                                                // if fastest to ball
      Log.log( 100, "I am fastest to ball; can get there in %d cycles", iTmp );
      soc = intercept( false );                      // intercept the ball

      if( soc.commandType == CMD_DASH &&             // if stamina low
          WM->getAgentStamina().getStamina() <
             SS->getRecoverDecThr()*SS->getStaminaMax()+200 )
      {
        soc.dPower = 30.0 * WM->getAgentStamina().getRecovery(); // dash slow
        ACT->putCommandInQueue( soc );
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
      else                                           // if stamina high
      {
        ACT->putCommandInQueue( soc );               // dash as intended
        ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
      }
     }
     else if( posAgent.getDistanceTo(WM->getStrategicPosition()) >
                  1.5 + fabs(posAgent.getX()-posBall.getX())/10.0)
                                                  // if not near strategic pos
     {
       if( WM->getAgentStamina().getStamina() >     // if stamina high
                            SS->getRecoverDecThr()*SS->getStaminaMax()+800 )
       {
         soc = moveToPos(WM->getStrategicPosition(),
                         PS->getPlayerWhenToTurnAngle());
         ACT->putCommandInQueue( soc );            // move to strategic pos
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
       else                                        // else watch ball
       {

         ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
         ACT->putCommandInQueue( turnNeckAtualiza_MatrizQToObject( OBJECT_BALL, soc ) );
       }
     }
     else if( fabs( WM->getRelativeAngle( OBJECT_BALL ) ) > 1.0 ) // watch ball
     {
       ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
       ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
     }
     else                                         // nothing to do
       ACT->putCommandInQueue( SoccerCommand(CMD_TURNNECK,0.0) );
   }


/*	
       if ( !WM->isBeforeKickOff() && WM->AR_getBallRelDistance() == 0){
			soc = AR_SafeDribble () ;
		    ACT->putCommandInQueue( soc ) ;}
	   else if( WM->AR_getBallRelDistance() < 3 ){
			soc = intercept (false) ;
            ACT->putCommandInQueue( soc ) ; }

       else if( posAgent.getDistanceTo(WM->getStrategicPosition()) >
                  1.5 + fabs(posAgent.getX()-posBall.getX())/10.0)
                                                  // if not near strategic pos
       {
       if( WM->getAgentStamina().getStamina() >     // if stamina high
                            SS->getRecoverDecThr()*SS->getStaminaMax()+800 )
       {
         soc = moveToPos(WM->getStrategicPosition(),
                         PS->getPlayerWhenToTurnAngle());
         ACT->putCommandInQueue( soc );            // move to strategic pos
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
       else                                        // else watch ball
       {
         ACT->putCommandInQueue( soc = turnBodyToObject( OBJECT_BALL ) );
         ACT->putCommandInQueue( turnNeckToObject( OBJECT_BALL, soc ) );
       }
     }
 */

/*	if( WM->getPlayerNumber() == 4 )
	{
        int i5 = WM->AR_getAngleChavei5() ;
		cerr << "Pele falando: A distancia do oponente: " << i5 << endl;
	}




			int i1 = WM->AR_getBallPos() ;
			int i2 = WM->AR_getBallRelDistance() ;
			int i3 = WM->AR_getCloTeammateRelDist() ;
			int i4 = WM->AR_getCloOpponentRelDist() ;
			int i5 = WM->AR_getAngleChavei5() ;
			int i6 = WM->AR_getRelativeOppAng() ;
			int i7 = WM->AR_getAgentGloBodyAngle() ;

			cerr << "Pele falando: A posicao da bola: " << i1 << endl;
			cerr << "Pele falando: A distancia a bola: " << i2 << endl;
			cerr << "Pele falando: A distancia do aliado: " << i3 << endl;
			cerr << "Pele falando: A distancia do oponente: " << i4 << endl;
			cerr << "Pele falando: O Angulo entre eles e a bola: " << i5 << endl;
			cerr << "Pele falando: O angulo do oponente: " << i6 << endl;
			cerr << "Pele falando: O meu angulo: " << i7 << endl;

			cerr << "Pele falando: Isso e tudo pessoal" << endl;



	}*/


void Player::Atualiza_MatrizQ( int reforco, int estadoFinal, int estadoAcao, int acao)
{
	int r;
	double delta, dQ, desconto, coef1, coef2;
	r = MA->RetornaReforco( acao, estadoAcao );

	delta = r + desconto*MA->RetornaMaxQ(estadoFinal) ;
	//Coeficiente de decaimento ajustado para
	// 1/((ln iteracao)/10000)
	coef1 = ((1/(log((AR->Iteracao_cont/24000.0) + 2.72))));
	coef2 = (-1)*WM->getGoalDiff( )/5.0 ;
	if( coef2 < 0.0 ) coef2 = 0.0 ;
	//if( coef1 < coef2 ) coef1 = coef2 ;
	coef1 = coef1 + coef2 ;
	if( coef1 > 1 ) coef1 = 1 ;
	dQ = coef1*delta ;

	MA->InsereMatrizQ( acao, estadoAcao, (1-coef1)*MA->RetornaValorQ(acao, estadoAcao) + dQ ) ;
	// (1-coef1)*Q(s,a) + coef1*(reforco + desconto*maxQ(s:))
	MA->InsereTransicao( acao, estadoAcao, estadoFinal ) ;
	AR->Iteracao_cont++ ;
}
