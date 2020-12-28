/*

License

Menge
Copyright © and trademark ™ 2012-14 University of North Carolina at Chapel Hill. 
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation 
for educational, research, and non-profit purposes, without fee, and without a 
written agreement is hereby granted, provided that the above copyright notice, 
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North 
Carolina at Chapel Hill. The software program and documentation are supplied "as is," 
without any accompanying services from the University of North Carolina at Chapel 
Hill or the authors. The University of North Carolina at Chapel Hill and the 
authors do not warrant that the operation of the program will be uninterrupted 
or error-free. The end-user understands that the program was developed for research 
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS 
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS 
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE 
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY 
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY 
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND 
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS 
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the authors {menge,geom}@cs.unc.edu

*/

#include "GoalSelectors/GoalSelectorNearest.h"
#include "Goals/Goal.h"
#include "GoalSet.h"
#include "BaseAgent.h"
#include <cassert>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>





namespace Menge {

	namespace BFSM {

		/////////////////////////////////////////////////////////////////////
		//                   Implementation of NearestGoalSelector
		/////////////////////////////////////////////////////////////////////
		
		Goal * NearestGoalSelector::getGoal( const Agents::BaseAgent * agent ) const  //se recibe como parámetro al agente
		{
			assert( agent != 0x0 && "NearestGoalGenerator requires a valid base agent!" );
			const size_t GOAL_COUNT = _goalSet->size(); //se le pasa a la variable GOAL_COUNT EL TAMAÑO DEL GOAL SET
			if ( GOAL_COUNT == 0 )//significa que no hay goles dentro del goal set
			{
				logger << Logger::ERR_MSG << "NearestGoalSelector was unable to provide a goal for agent " << agent->_id << ".  There were no available goals in the goal set.";
				return 0x0;
			}
			//de aqui en adelante significa que si hay goals dentro del goal set

			const Vector2 p = agent->_pos; // a la variable p de tipo de dato Vector2, se le asigna la posicion x,y del agente

			Goal * bestGoal; // se declara el puntero bestGoal, del tipo de dato Goal

			srand((unsigned)time(NULL)); //crea la semilla
			//int sensor_activado=(rand()%4);  //se genera aleatoriamente el sensor de la puerta activado
			
			int sensor_activado=0;

		

            
		
			
			if(sensor_activado==0)//en la primer puerta se encendió el sensor
			{
				bestGoal = _goalSet->getIthGoal( 1 ); //a la variable bestGoal se le asigna por default el primer goal del goal set (el 0)
				//bestGoal= _goalSet->getGoalByID(5) 
						
				Vector2 disp = bestGoal->getCentroid()-p;//se le asigna a la variable disp el valor resultante del vector del nodo - p(pos)
				float bestDist = absSq( disp );//se le saca la raiz cuadrada a disp y se guarda el valor en la variable bestDist

						

				for ( size_t i = 2; i < GOAL_COUNT; ++i ) //se inicia un ciclo para recorrer todos los nodos
				{
					
						Goal * testGoal = _goalSet->getIthGoal( i );//se declara la variable testGoal qu es un puntero, del tipo de dato
						//Goal, para ir haciendo las pruebas necesarias a medida que se vayan recorriendo los nodos en el ciclo

						disp = testGoal->getCentroid() - p;//se le asigna a la variable disp el valor resultante del vector del nodo(i) - p(pos)
						float testDist = absSq( disp );//se le saca la raiz cuadrada a disp y se guarda el valor en la variable testDist 
								

						//aqui se le tiene que agregar que sea segura la ruta
						if ( testDist < bestDist)//se testDist es menor a la distancia que se consideraba la menor entonces...
						{
							bestDist = testDist; //a la mejor distancia(menor) se le asigna la de la prueba actual en el ciclo
							bestGoal = testGoal;// eso implica que ese sea el mejor goal
						}

					
							
				}

			}
			else
			{
					bestGoal = _goalSet->getIthGoal( 0 ); //a la variable bestGoal se le asigna por default el primer goal del goal set (el 0)
					//bestGoal= _goalSet->getGoalByID(5) 
							
					Vector2 disp = bestGoal->getCentroid()-p;//se le asigna a la variable disp el valor resultante del vector del nodo - p(pos)
					float bestDist = absSq( disp );//se le saca la raiz cuadrada a disp y se guarda el valor en la variable bestDist

							

					for ( size_t i = 1; i < GOAL_COUNT; ++i ) //se inicia un ciclo para recorrer todos los nodos
					{
						//printf("\nsensor activado: %d",sensor_activado);
						//printf("\ni: %d\n",i);
						if(i!= sensor_activado)//es seguro
						{
							Goal * testGoal = _goalSet->getIthGoal( i );//se declara la variable testGoal qu es un puntero, del tipo de dato
							//Goal, para ir haciendo las pruebas necesarias a medida que se vayan recorriendo los nodos en el ciclo

							disp = testGoal->getCentroid() - p;//se le asigna a la variable disp el valor resultante del vector del nodo(i) - p(pos)
							float testDist = absSq( disp );//se le saca la raiz cuadrada a disp y se guarda el valor en la variable testDist 
									

							//aqui se le tiene que agregar que sea segura la ruta
							if ( testDist < bestDist)//se testDist es menor a la distancia que se consideraba la menor entonces...
							{
								bestDist = testDist; //a la mejor distancia(menor) se le asigna la de la prueba actual en el ciclo
								bestGoal = testGoal;// eso implica que ese sea el mejor goal
							}

						}
								
					}

			}

			

		
		
		

			return bestGoal; //se retorna el mejor goal para ese agente
		}
	}	// namespace BFSM
}	// namespace Menge
