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

#include "SimSystem.h"
// SceneGraph
#include "GLScene.h"
// MengeBase
#include "SimulatorInterface.h"
#include "SpatialQueries/SpatialQuery.h"
#include "Obstacle.h"
#include "SCBWriter.h"
// MengeRuntime
#include "VisAgent.h"
#include "VisObstacle.h"
// BFSM
#include "FSM.h"
// STL
#include <set>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>


namespace Menge {

	////////////////////////////////////////////////////////////////////////////
	//			Implementation of SimSystem
	////////////////////////////////////////////////////////////////////////////

	SimSystem::SimSystem( bool visualize ): SceneGraph::System(), _forVis(visualize), _sim(0x0), _fsm(0x0), _scbWriter(0x0),_lastUpdate(0.f), _isRunning(true), _maxDuration(100.f) {
	}

	////////////////////////////////////////////////////////////////////////////

	SimSystem::SimSystem( bool visualize, float duration ): SceneGraph::System(), _forVis(visualize), _sim(0x0), _fsm(0x0), _scbWriter(0x0),_lastUpdate(0.f), _isRunning(true), _maxDuration(duration) {
	}

	////////////////////////////////////////////////////////////////////////////

	SimSystem::~SimSystem() {
			if ( _sim ) delete _sim;
			if ( _fsm ) delete _fsm;
			if ( _scbWriter ) delete _scbWriter;
		}

	////////////////////////////////////////////////////////////////////////////

	bool SimSystem::updateScene( float time ) {
		const int agtCount = static_cast<int>( _sim->getNumAgents() );
		if ( _isRunning ) {
			if ( _scbWriter ) _scbWriter->writeFrame( _fsm );	
			_lastUpdate = _sim->getGlobalTime();
			if ( _lastUpdate > _maxDuration ) {
				_isRunning = false;
			} else {
				for ( size_t i = 0; i <= _sim->getSubSteps(); ++i ) {
					try {
						_isRunning = !_fsm->doStep();
					} catch ( BFSM::FSMFatalException & e ) {
						logger << Logger::ERR_MSG << "Error in updating the finite state machine -- stopping!\n";
						logger << "\t" << e.what() << "\n";
						throw SceneGraph::SystemStopException();
					}

					_sim->doStep();
					if ( _forVis ) {
						updateAgentPosition( agtCount );
					}
					try {
						_fsm->doTasks();
					} catch ( BFSM::FSMFatalException &e ) {
						logger << Logger::ERR_MSG << e.what() << "\n";
						throw SceneGraph::SystemStopException();
					}
				}
			}
		}
		if ( !_isRunning ) {
			// TODO: WHy is this here??
			throw SceneGraph::SystemStopException();
			return false;
		}
		return true;
	}

	////////////////////////////////////////////////////////////////////////////

	bool SimSystem::isFinished() const { 
		return _fsm->allFinal(); 
	}

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::setSimulator( Agents::SimulatorInterface * sim, BFSM::FSM * fsm ) {
		if ( _sim ) {
			std::string msg( "Simulator already assigned to SimSystem" );
			throw SimSystemFatalException( msg );
		}
		_sim = sim;
		_fsm = fsm;
	}

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::setSimulator( Agents::SimulatorInterface * sim, BFSM::FSM * fsm, const std::string & outFileName, const std::string & scbVersion ) {
		setSimulator( sim, fsm );
		try {
			_scbWriter = new Agents::SCBWriter( outFileName, scbVersion, sim );
		} catch ( Agents::SCBFileException ) {
			std::string msg( "Unable to create SCB file: ");
			msg += outFileName;
			throw SimSystemFatalException( msg );
		}
	}

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::addObstacleToScene( SceneGraph::GLScene * scene ) {
			// TODO: If the bsptree (ObstacleKDTree.h) chops up the obstacles, this isn't doing the
			//		right thing.  Currently, the bsptree chops them
			//	THIS IS A HACK to address the issues of the ObstacleKDTree
			//		The right thing to do is modify things so that they are not chopped up.
			std::set< const Agents::Obstacle * > handled;
			const std::vector< Agents::Obstacle * > & obstacles = _sim->getSpatialQuery()->getObstacles();
			for ( size_t o = 0; o < obstacles.size(); ++o ) 
			{
				
				const Agents::Obstacle * obst = obstacles[ o ];
				if ( handled.find( obst ) == handled.end() )
				{
					Vector2 p0a = obst->getP0();
					Vector2 p1a = obst->getP1();
					const Agents::Obstacle * next = obst->_nextObstacle;
					while ( next && next->_unitDir * obst->_unitDir >= 0.99999f )
					{
						handled.insert( next );
						p1a.set( next->getP1() );
						next = next->_nextObstacle;
					}
					Vector3 p0( p0a.x(), _sim->getElevation( p0a ), p0a.y() );
					Vector3 p1( p1a.x(), _sim->getElevation( p1a ), p1a.y() );
					VisObstacle * vo = new VisObstacle( p0, p1 );
					
					scene->addNode( vo );
					handled.insert( obst );
				}
			}
		}
	////////////////////////////////////////////////////////////////////////

	int poisson(float e_menos_lambda) 
	{
		int n = 0;
		
		
		//printf("\ne_menos_lambda: %.3f",e_menos_lambda);

		
		float ri, t=1;
		
		time_t tt;

		srand(time(NULL)*rand()/1000000);

		while (t >= e_menos_lambda)
		{
			//srand(getpid());
			
			ri = ((double)rand()/100000);
			
		//	printf("\nri: %.3f",ri);
		//	printf("\nn: %d",n);

			if (t < e_menos_lambda) 
			{
				break;
			}
			else 
			{
				n=n+1;
				t = t*ri;

			}

		} 



		return n;
	}

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::addAgentsToScene( SceneGraph::GLScene * scene ) 
	{
		_visAgents = new VisAgent * [ _sim->getNumAgents() ];



		
		std::ofstream fs("SimulacionDeColas.txt");

  
   

		//fs << _sim->getNumAgents() <<std::endl; //imprime la totalidad de agentes en la simulacion

		//time_t ttt;
		//srand((unsigned) time(&ttt));

		

		double e_menos_lambda = exp(-3.0); //lamda==1

		int tiempo = 0;
		char tipo_evento[15];
		int n_cliente=0;
		int n_cliente_salida=0;
		char estado_servidor[100];
		int long_cola=0;
		float na1=0;
		char lle[100];
		float na2=0;
		char sal[100];
		char lista_eventos[100];
		int min_act=0;
		int min_act1=0;
		int auxi=0;
		char proximo_evento[3];
		char proximo_evento1[3];
		int nuevo_tiempo=0;
		int nuevo_tiempo1=0;
		char buffer[100];
		char zicri[100];
		int x=0;
		char peter[100];
		char xime[100];
		char savi[100];
		char sara[100];
		 char gg[100];
		 char nel[100];

		strcpy(lista_eventos,"E1/0;E3/30\0");
		strcpy(estado_servidor,"libre");

		fs << "RELOJ\t\tTIPO EVENTO\t\tN° CLIENTE\t\tESTADO SERVIDOR\t\tLONG COLA\t\tNA1\t\tLLE\t\tNA2\t\tSAL\t\tLISTA EVENTOS" << std::endl;//imprime la cabecera de la tabla de simulación
		fs << "---------------------------------------------------------------------------------------------------------------------------------------" << std::endl;	
		fs << "0\t\tinicial\t\t\t\tO\t\t\tlibre\t\t0\t\t\t---\t\t---\t\t---\t\t---\t\tE1/0;E3/30" << std::endl;//imprime el estado inicial de simulación
		 
		  
		char *ptr,*ptr1,eventoo[5],auxp[15],fg[100];
		
		
		
		while (tiempo<30) //se simula en base a 30 min ocupados en la pizzeria
		{
			strcpy(savi,lista_eventos);
			printf("lista: %s-----------------------------------\n",lista_eventos);
		
			ptr = strtok(lista_eventos,";");//obtiene el primer elemento de la lista
			strcpy(auxp,ptr);
			
			ptr1=0;
			ptr1 = strtok(auxp,"/");//obtiene el tipo de evento (E1 o E2 o E3)
			strcpy (eventoo,ptr1);
			printf("evento a analizar: %s\n",eventoo);
			
			//strcat (str,"strings ");
			ptr1 = strtok(NULL, "/");//obtiene el tiempo de ese evento
			min_act = atoi(ptr1);
			
			
			

			tiempo=min_act; //tiempo de simulación actual debido al evento actual
			printf("tiempo: %d\n",tiempo);
			strcpy(sal,"");
			strcpy(lle,"");
			
			printf("\nlong_cola: %d\n",long_cola);
			printf("\nestado_servidor: %s\n",estado_servidor);
		
			
			if(strcmp(eventoo,"E1")==0)//es una llegada
			{
					
				
				n_cliente++;
				strcpy(tipo_evento,"llegada");
				
				tiempo=min_act; //tiempo de simulación actual debido al evento actual

				if(strcmp(estado_servidor,"libre")==0)//está libre
				{
					x=1;
					strcpy (estado_servidor,"ocupado");

					//generar E2 y tE2= reloj+SAL
					strcpy(proximo_evento,"");
					strcpy (proximo_evento,"E2");
					strcat (proximo_evento,"/");
					nuevo_tiempo1 = poisson(e_menos_lambda) + tiempo;
					
					
					
					na2=nuevo_tiempo1;
					itoa (nuevo_tiempo1,buffer,10);
					
					

					
					strcat (proximo_evento,buffer);
					strcat(proximo_evento,"\0");
					strcpy (sal,proximo_evento);
					
					strcpy (nel,proximo_evento);
					strcpy(proximo_evento,"");
					
					

					
					

				}
				else//está ocupado
				{
					x=3;
					
					long_cola++;
					strcpy (sal,"---\0");
					nuevo_tiempo1=-1;
					
					
					
				}

				//generar E1 y tE1= reloj+LLE
				strcpy(proximo_evento,"");
				
				strcpy (proximo_evento,"E1");
				nuevo_tiempo = poisson(e_menos_lambda) + tiempo;
				
				na1= (float)nuevo_tiempo;

				itoa(nuevo_tiempo,buffer,10);

				strcat (proximo_evento,"/");
				strcat (proximo_evento,buffer);
				strcat(proximo_evento,"\0");
				strcpy (lle,proximo_evento);
				strcpy(proximo_evento,"");
				

				
			}
			else if(strcmp(eventoo,"E2")==0)//es una salida
			{
				
				strcpy(tipo_evento,"salida");
				n_cliente_salida++;
				

				tiempo=min_act; //tiempo de simulación actual debido al evento actual

				if(long_cola>0)
				{
					
					x=2;
					long_cola--;
					na1=-1;
					nuevo_tiempo=-1;
					strcpy (lle,"---");

					//generar E2 y tE2= reloj+SAL
					strcpy (proximo_evento,"");
					strcpy (proximo_evento,"E2");
					nuevo_tiempo1 = poisson(e_menos_lambda) + tiempo;
					
					na2=(float)nuevo_tiempo1;
					itoa (nuevo_tiempo1,buffer,10);
					
					strcat (proximo_evento,"/");
					strcat (proximo_evento,buffer);
					strcat(proximo_evento,"\0");
					strcpy (sal,proximo_evento);
					strcpy (nel,proximo_evento);
					
					
					//printf("\n\nsalllllllllllllllllllll::::::::::  %s\n\n",sal);

				}
				else
				{
				//	printf("entra");
					x=4;
					strcpy (estado_servidor,"libre");
					strcpy (lle,"---\0");
					strcpy (sal,"E2");
					strcat(sal,"/");
					itoa (tiempo,buffer,10);
					strcat (sal,buffer);
					strcat(sal,"\0");
					nuevo_tiempo1=-1;
					nuevo_tiempo=-1;
				}

			}
			else//fin de la simulación
			{
				strcpy(tipo_evento,"final");
				
				if(tiempo>=30)
			    	break;
			}
		

			//actualizar lista de eventos-------------
			
			strcpy(lista_eventos,savi);
			
			
			ptr=0;

			ptr = strtok(lista_eventos,";");//obtiene el primer elemento de la lista++
            
			strcpy(zicri,"");
			strcpy(auxp,"");
			
			//ptr = strtok(NULL,";");//obtiene el primer elemento de la lista++
			

			while(ptr != NULL)//recorrer la lista de eventos hasta el final
			{
				
			 
			  
			  if(ptr!= NULL)
			  {
			  	if(strcmp(ptr,"E3/30")==0)
				  {
				  	
				 
				    //printf("ptr: %s\n",ptr);
				   // printf("%s\n",ptr);
				  	
				  	strcpy(auxp,ptr);
				  	
				    strcat (zicri,auxp);//voy almacenando lo que queda de lista
				    ptr = strtok(NULL, ";");
				   
				  	break;
				  }
				  else
				  {
				  	
				  	  //printf("ptr: %s\n",ptr);
					  	strcpy(auxp,ptr);
					  	
					  	ptr = strtok(NULL, ";");
					   // printf("ptr: %s\n",ptr);
					   
					   
					   if(ptr!=NULL)
					   {
					   	strcat (zicri,auxp);//voy almacenando lo que queda de lista
					    strcat (zicri,";");//le anexo el ; que se quitó
					   	
					   }
					
					    
			
				  	
				  }
			  	
			  	
			  }
			  
			 
			}
			
			
			
			
			
			
			
			
			/*strcpy(fg,"");
					
			memcpy(fg,lista_eventos,strlen(lista_eventos-5));					
			strcat(fg,"E3/30\0");
			
			printf("\n\n\n\nfg::::  %s\n\n",fg);
			
			strcpy(zicri,fg);*/
			
			strcpy(sara,"");
			
			strcpy(sara,zicri);
			
			
		
			
			ptr = strtok(sara,";");//obtiene el primer elemento de la lista ya quitada su primer valor
		    ptr = strtok(NULL,";");//obtiene el primer elemento de la lista ya quitada su primer valor
		
			strcpy(xime,ptr);
			//strcpy(auxp,ptr);
			ptr1=0;
			ptr1 = strtok(xime,"/");//obtiene el tipo de evento (E1 o E2 o E3)
			strcpy (eventoo,ptr1);
			//strcat (str,"strings ");
			ptr1 = strtok(NULL, "/");//obtiene el tiempo de ese evento
			min_act1 = atoi(ptr1);//tiempo que verificaré si es menor o mayor al que solicito
			
	
			
			strcpy(peter,"");
			
		
			
			if(x==1)//entran llegadas y salidas
			{
				
				strcpy(sal,nel);

				if(nuevo_tiempo1<= nuevo_tiempo)//va primero la salida
				{
					strcpy (peter,sal);
					strcat(peter,";");
					strcat(peter,lle);
					strcat(peter,";");
					strcat(peter,"E3/30\0");//le anexo la parte final de la lista
					//strcat(peter,zicri);

				}
				else//va primero la entrada
				{

					strcpy (peter,lle);
					strcat(peter,";");
					strcat(peter,sal);
					strcat(peter,";");
					strcat(peter,"E3/30\0");//le anexo la parte final de la lista
					//strcat(peter,zicri);

				}

			}
			else if(x==2)//entran salidas
			{
				
				strcpy(peter,"");
				strcpy(sal,nel);

				if(nuevo_tiempo1<min_act1 && nuevo_tiempo1!=-1)//va al inicio
				{
					strcpy (peter,sal);
					
					strcat(peter,";");
					
				//	strcat(peter,lista_eventos);
				//	strcat(peter,";");
					
					
					strcat(peter,"E3/30");
				    strcat(peter,"\0");
				
				 //strcat(peter,zicri);
				
					

				}
				else//va en sugundo lugar
				{
				  
				  strcpy (peter,xime);
				  strcat(peter,"/");
				  itoa (min_act1,buffer,10);
				  strcat(peter,buffer);
				  strcat(peter,";");
				  strcat(peter,sal);
				  strcat(peter,";");
				  
				 // strcat(peter,lista_eventos);
				  //strcat(peter,";");
				  
				  strcat(peter,"E3/30\0");
				  //strcat(peter,zicri);
				}

			}
			else if(x==3)//entran entradas
			{
			
				

				if(nuevo_tiempo1<min_act1 && nuevo_tiempo1!=-1)//va al inicio
				{
					strcpy (peter,lle);
					strcat(peter,";");
					//strcat(peter,zicri);//le anexo la parte final de la lista
					strcat(peter,"E3/30\0");
				

				}
				else//va en sugundo lugar
				{
				  strcpy (peter,xime);
				   strcat(peter,"/");
				  itoa (min_act1,buffer,10);
				  strcat(peter,buffer);
				  strcat(peter,";");
				  
				  strcat(peter,lle);
				  strcat(peter,";");
				  
				  strcat(peter,"E3/30\0");
				  //strcat(peter,zicri);
				}

			}
			else
			{   
			    
			   
			    //strcpy(peter,lista_eventos);
			    itoa (min_act1,gg,10);
				strcpy(peter,xime);
				strcat(peter,"/");
				strcat(peter,gg);
				strcat(peter,";");
			    strcat(peter,"E3/30\0");
			    //strcat(peter,zicri);
		   
				
				
				//break;
			}
			  
			
		/*	printf("peter: %s\n",peter);
			printf("tiempo:%d\n",tiempo);
			printf("llegada:%s\n",lle);
			printf("salida:%s\n",sal);*/

			strcpy(lista_eventos,peter);
		//	printf("%s",lista_eventos);
			strcpy(peter,"");
			strcpy(zicri,"");
			strcpy(xime,"");
			strcpy(buffer,"");
		
			


			fs << tiempo<<"\t\t"<<tipo_evento<<"\t\t\t\t"<<n_cliente<<"\t\t\t"<<estado_servidor<<"\t\t"<<long_cola<<"\t\t\t"<<na1<<"\t\t"<<lle<<"\t\t"<<na2<<"\t\t"<<sal<<"\t\t"<<lista_eventos<<std::endl;
		}

		
		fs.close();


		

		for ( size_t a = 0; a < _sim->getNumAgents(); ++a ) 
		{
			Agents::BaseAgent * agt =  _sim->getAgent( a );
			VisAgent * agtNode = new VisAgent( agt );
			float h = _sim->getElevation( agt );
			agtNode->setPosition( agt->_pos.x(), h, agt->_pos.y() );
			scene->addNode( agtNode );
	

			//fs << a <<std::endl;//imprime el id del agente
			////fs << agt->_pos.x() <<std::endl; //imprime el punto en el eje x
			//fs << h <<std::endl; //imprime la elevacion del agente
			//fs << agt->_pos.y() <<std::endl; //imprime el punto en el eje y
			
			float orient = 999;
			
			//fs <<orient <<std::endl; //imprime la orientacion del agente


			//modelo socio-cultural-------------------------------------------------------------

			float religiosidad,educacion,nivelSocial,nivelSociable,Region,idiomas,nivelSolidario,totallll;
   
   
   
   			religiosidad=(float)((rand()%10)+1)/10;
			educacion=(float)((rand()%10)+1)/10;
			nivelSocial=(float)((rand()%10)+1)/10;
			nivelSociable=(float)((rand()%10)+1)/10;
			Region=(float)((rand()%10)+1)/10;
			idiomas=(float)((rand()%10)+1)/10;
			nivelSolidario=(float)((rand()%10)+1)/10;


			 //parte de las variables del modelo de velocidad pedestre

			float e,hh,w,cal,inc,sum,c;
			//srand(time(NULL));
			
			c=(float)(rand()%10)+1;
			e=(float)((rand()%10)+1)/10;
			hh=(float)((rand()%10)+1)/10;
			w=(float)((rand()%10)+1)/10;
			cal=(float)((rand()%10)+1)/10;
			inc=(float)((rand()%10)+1)/10;

			sum=(hh+w+cal)/3;

		    
			if(idiomas>0.5)
			   idiomas=1;
			else
				idiomas=0.5;
					
		   totallll= pow(religiosidad*educacion*nivelSocial*nivelSociable*Region*idiomas,nivelSolidario);

			float fa=totallll;
   
			if(fa>0.01 && fa<0.1)
            {
              fa=fa*10;
            }
            else if(fa>0.001 && fa<= 0.009)//0.001
            {
                 fa=fa*100;
            }
            else if(fa<0.001)
                 fa= 0.1f;

			agt->_maxAccel=fa/w;

            if (agt->_maxAccel < 0.09 && agt->_maxAccel > 0.01)
               agt->_maxAccel = agt->_maxAccel * 10;
            else if (agt->_maxAccel > 0 && agt->_maxAccel < 1)
                agt->_maxAccel = agt->_maxAccel * 10;


			

			agt->_acompanantes=(int)c;
			agt->_edad=(float)e;
			agt->_incapacidad=(float)inc;
			agt->_sum=(float)sum;

			//f= m*a (segunda Ley de Newton) para el modelo socio-cultural   -->    a= f/m
            agt->_maxAccel =(float) fa / w;


			if (agt->_maxAccel < 0.09 &&agt->_maxAccel > 0.01)
                agt->_maxAccel = agt->_maxAccel * 100;
            else if (agt->_maxAccel > 0 && agt->_maxAccel < 1)
                agt->_maxAccel = agt->_maxAccel * 10;
			else if(agt->_maxAccel < 0.01)
				agt->_maxAccel = agt->_maxAccel * 1000;



			_visAgents[ a ] = agtNode;

		}
		//fs.close();
	}


	

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::populateScene( SceneGraph::GLScene * scene ) {
		assert( _sim !=  0x0 && "Can't add SimSystem to scene when no simulator is connected" );

		addAgentsToScene( scene );
		addObstacleToScene( scene );
	}

	////////////////////////////////////////////////////////////////////////////

	void SimSystem::updateAgentPosition( int agtCount ) {
		//#pragma omp parallel for
		/*for ( int a = 0; a < agtCount; ++a ) 
		{
			//float fuck= (float)(rand()%10)+1;
			Agents::BaseAgent * agt = _visAgents[ a ]->getAgent();
			float h = _sim->getElevation( agt );
			_visAgents[ a ]->setPosition( agt->_pos.x(), h, agt->_pos.y() );
			
		
		}
		*/

		time_t ttt;
		srand((unsigned) time(&ttt));

		for(size_t a=0; a<_sim->getNumAgents();a++)
		{
			//ecuacion de velocidad----------------
			Agents::BaseAgent * agtt=_sim->getAgent( a );
			
			//modelo de velocidad pedestre-----------------------------------------------------


			float sec= _sim->getGlobalTime();
            
			float totalll,k;
			
			k=1/sec;
			
			
			if(k<0.1 && k>=0.01)
			{
				k=k*10-0.1f;
			}


			totalll= ((agtt->_edad*k*agtt->_sum*agtt->_incapacidad)/agtt->_acompanantes);

			if(totalll>0.01 && totalll<0.1)
            {
              totalll=totalll*10;
            }
            else if(totalll>0.001 && totalll<= 0.009)//0.001
            {
                 totalll=totalll*100;
            }
            else if(totalll<0.001)
                 totalll=(float)0.1;

			
			agtt->_velPref.setSpeed(totalll+1);

			


			float h = _sim->getElevation( agtt );
			_visAgents[ a ]->setPosition( agtt->_pos.x(), h, agtt->_pos.y() );

		}

		
	}

	////////////////////////////////////////////////////////////////////////////

	size_t SimSystem::getAgentCount() const { 
		return _sim->getNumAgents(); 
	}

	////////////////////////////////////////////////////////////////////////////

}	// namespace Menge