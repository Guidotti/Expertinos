/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * Time_Base2
/bin/bash: line 1: cd: /home/felipe/Documents/Iniciação: No such file or directory
/bin/bash: line 62: cd: /home/felipe/Documents/Iniciação: No such file or directory
 * Copyright (C) Felipe Lira Santana Silva 2012 <felipe.lira.santana@gmail.com>
 * 
Time_Base2 is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Time_Base2 is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ARMatrizes.h"




#include <iostream>
using namespace std;
#include <fstream>
#include <cstdlib>

AR_Matrizes::AR_Matrizes(){}
AR_Matrizes::~AR_Matrizes(){}
/* Este método inicializa a matriz de estados e salva em um arquivo csv*/
void AR_Matrizes::inicializaEstados()
{
    int i = 0;// contador da linha da matriz
	int temp_j = inum - 1;// Necessário para definir qual termo será atualizado
	int j = 0 ;
	int tempChaves[inum] ;
	//Inicialização do vetor
	for( i=0; i < inum; i++ )
		tempChaves[i] = 0 ;

	//Inicialização da matriz
	for( i=0, i < numEstados, i++ )
	{
		temp_j = inum - 1 ;
		//Verifica qual indice do vetor temp_j será atualizado
		while( tempChaves[temp_j] > (chaves[temp_j] - 1) )
		{
			tempChaves[temp_j] = 0
			temp_j-- ;
		}
		tempChaves[temp_j]++ ;
			
		for( j=0, j < inum, j++ )
		{
			S[i][j] = tempChaves[j] ;
		}
	} // Finalizada a matriz

	//Salva a matriz em arquivo
	ofstream MatrizEstados( "MatrizEstados.dat", ios::out ) ;
	if( !MatrizEstados )
	{
		cerr << "Arquivo nao pode ser aberto" << endl;
		exit( 1 ) ;
	}

	for( i=0, i < numEstados, i++ )
	{
		for( j=0, j < inum, j++ )
		{
			MatrizEstados << S[i][j] ;
		}
		MatrizEstados << '\n' ;
	}
	return 0 ;
}
