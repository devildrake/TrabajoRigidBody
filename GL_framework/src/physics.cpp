#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <cassert>
#include "GL_framework.h"
#include <iostream>
#include <time.h>
namespace Box {
	extern void setupCube();
	extern void cleanupCube();
	extern void drawCube();
}
namespace Cube {
	extern void  setupCube();
	extern void  drawCube();
	extern glm::mat4 modelMat;
	extern float cubeVerts[];
}

bool collisions = false;
bool start = false;
bool slowMo = false;
bool restart = true;
bool show_test_window = false;
float epsilon;
float vertices[24];


glm::vec3 gravity = glm::vec3(0,-9.8,0);

class BoxObj {
public:

	class Plane {
	public:
		float d;
		glm::vec3 n;
		Plane() {
			d = n.x = n.y = n.z = 0;
		}
		Plane(float nx, float ny, float nz, float d) {
			this->n.x = nx;
			this->n.y = ny;
			this->n.z = nz;
			this->d = d;
		}
		void SetPlaneStats(float nx, float ny, float nz, float d) {
			n.x = nx;
			n.y = ny;
			n.z = nz;
			this->d = d;
		}
	};
	Plane planos[6];


	//Constantes
	glm::mat3 iBody;
	float mass;
	//Variables
	glm::vec3 orgPos;
	glm::mat4 prevMod;
	glm::vec3 pos;
	glm::quat quaternion;
	glm::vec3 linealMom;
	glm::vec3 angMom;
	glm::vec3 torque;
	glm::mat3 rotation;
	int escalado;
	glm::mat3 inverseInertia;
	glm::vec3 verticesAnteriores[8];
	glm::vec3 verticesActuales[8];
	//REINICIO
	void Restart(float dt) {

		linealMom = angMom = glm::vec3(0);

		pos = orgPos;

		srand(time(NULL));
		glm::vec3 tempForce;
		tempForce.x = (float)rand() / RAND_MAX * 1000 - 500;
		tempForce.y = (float)rand() / RAND_MAX * 2000;
		tempForce.z = (float)rand() / RAND_MAX * 1000 - 500;
		linealMom = linealMom + ((gravity*mass) + tempForce)*dt;

		glm::vec3 forcePos;

		//(float)rand() / RAND_MAX;
		forcePos.x = (float)rand() / RAND_MAX * 2 - 1;
		forcePos.y = (float)rand() / RAND_MAX * 2 - 1;
		forcePos.z = (float)rand() / RAND_MAX * 2 - 1;

		//std::cout << tempForce.x << " " << tempForce.y << " " << tempForce.z << "\n";

		torque = glm::cross(forcePos-pos,tempForce);
		angMom = angMom+torque*dt;

		torque = glm::vec3(0);
	}

	//CONSTRUCTOR
	BoxObj(glm::vec3 orgPos, int s){
		this->orgPos = orgPos;
		pos = orgPos;
		escalado = s;
		mass = escalado;

		//iBody es la misma en las tres partes de la matriz porque es un cubo y los lados son iguales
		iBody[0][0] = iBody[1][1] = iBody[2][2] = (1.0f / 12.0f)*mass*(escalado*escalado + escalado*escalado);
		planos[0].SetPlaneStats(0.0f, 1.0f, 0.0f, 0.0f); // Parte abajo del cubo
		planos[1].SetPlaneStats(0.0f, 0.0f, -1.0f, -5.0f); //Parte del fondo
		planos[2].SetPlaneStats(-1.0f, 0.0f, 0.0f, 5.0f); //Parte izquierda
		planos[3].SetPlaneStats(0.0f, 0.0f, 1.0f, -5.0f); //Parte de delante
		planos[4].SetPlaneStats(1.0f, 0.0f, 0.0f, 5.0f); //Parte derechas
		planos[5].SetPlaneStats(0.0f, -1.0f, 0.0f, 10.0f); //Parte de arriba
	}

	//ACTUALIZACIÓN DEL PINTADO
	void UpdateDraw() {


		Cube::modelMat = glm::mat4(1.f);

		Cube::modelMat = glm::translate(Cube::modelMat, pos);

		prevMod = Cube::modelMat;


		glm::mat4 tempRot;

		tempRot = rotation;

		Cube::modelMat *= tempRot;

		Cube::modelMat = glm::scale(Cube::modelMat, glm::vec3(escalado, escalado, escalado));

	}

	void Bounce(Plane plano,float dt, glm::vec3 verticeEnContacto) {
		float velocidadRelativa;
		glm::vec3 derivadaPA;

		derivadaPA  = linealMom / mass + glm::cross(( inverseInertia * glm::vec3(angMom.x, angMom.y, angMom.z)),glm::vec3(Cube::modelMat*glm::vec4(verticeEnContacto,1)) - glm::vec3((Cube::modelMat*glm::vec4(0,0,0,1))));

		velocidadRelativa = glm::dot(plano.n,derivadaPA);

		glm::vec3 posicionDeVertice = glm::vec3(Cube::modelMat*glm::vec4(verticeEnContacto.x, verticeEnContacto.y, verticeEnContacto.z, 1.0f));

		glm::vec3 parentesis1 = glm::cross(posicionDeVertice,plano.n);
		glm::vec3 parentesis2 = inverseInertia*parentesis1;
		glm::vec3 parentesis3 = glm::cross(parentesis2,posicionDeVertice);

		float laJota = (-(1 + epsilon)*velocidadRelativa) / (1 / mass + glm::dot(plano.n,parentesis3));
		
		glm::vec3 vecJota = laJota*plano.n;

		glm::vec3 torqueNuevo = glm::cross(posicionDeVertice,vecJota);

		linealMom += vecJota;

		angMom += torqueNuevo;

	}

	void CheckCol(float dt, glm::vec3 prevVert[], glm::vec3 curVert[]) {
		for (int j = 0; j < 8; j++) {
			for (int i = 0; i < 6; i++) {
			
			if ((glm::dot(planos[i].n, prevVert[j])+planos[i].d)*(glm::dot(planos[i].n,curVert[j])+planos[i].d)<=0) {

				//glm::mat4 temp;

				simulateEulerStep(dt,i,j);

				Bounce(planos[i], dt, curVert[j]);

				//Cube::modelMat = temp;


				}
			}
		}
	}

	//ACTUALIZADO
	void Update(float dt) {
		if(slowMo)
		dt = dt / 8;


		UpdatePosAndRot(dt);
		int contador = 0;

		for (int i = 0; i < 8; i++){
			for (int j = 0; j < 3; j++) {
				vertices[contador] = Cube::cubeVerts[i * 6 + j];
				contador++;
			}
		}


		int contador2 = 0;
		for (int i = 0; i < 24; i+=3) {

//			for(int k = 0;k<3;k++){
//				for (int j = 0; j < 3; j++){
//				std::cout << prevMod[k][j] << " ";
//}
//				std::cout<<"\n";
//			}
			verticesAnteriores[contador2] = prevMod * (glm::vec4(vertices[i], vertices[i + 1], vertices[i + 2], 1));
			contador2++;
		}

		contador2 = 0;
		for (int i = 0; i < 24; i += 3) {
			
			//std::cout << verticesActuales[i].x << " " << verticesActuales[i].y << " " << verticesActuales[i].z << std::endl;

			verticesActuales[contador2] = Cube::modelMat * (glm::vec4(vertices[i], vertices[i + 1], vertices[i + 2], 1));
			contador2++;
		}

		//for (int i = 0; i < 24; i++) {
		//	//std::cout << verticesActuales[i].x << " " << verticesActuales[i].y << " " << verticesActuales[i].z << std::endl;

		//}

		for (int i = 0; i < 8; i++) {
			if (verticesActuales[i].x > 5) {
				float offsetX = 5 - verticesActuales[i].x;
			}
			else if (verticesActuales[i].x < -5) {
				float offsetX = -5 + verticesActuales[i].x;
			}

			if (verticesActuales[i].y > 5) {
				float offsetY = 5 - verticesActuales[i].y;

			}
			else if (verticesActuales[i].y < -5) {
				float offsetY =- 5 - verticesActuales[i].y;
			}
			if (verticesActuales[i].z > 5) {
				float offsetZ = 5 - verticesActuales[i].z;

			}	else if (verticesActuales[i].z < -5) {
				float offsetZ = - 5 - verticesActuales[i].z;

			}
		}

		if(collisions)
		CheckCol(dt,verticesAnteriores,verticesActuales);



		UpdateDraw();
	}



	void UpdatePosAndRot(float dt) {
		linealMom = linealMom + dt*(gravity*mass);
		angMom = angMom + dt*torque;
		glm::vec3 velocity = linealMom / mass;
		pos = pos + velocity*dt;
		inverseInertia = rotation * glm::inverse(iBody)*glm::transpose(rotation);
		glm::vec3 angularVelocity = inverseInertia * glm::vec3(angMom.x, angMom.y, angMom.z);
		//AL DECLARAR MATRICES EN GLM CON EL CONSTRUCTOR, RECIBE LA TRASPOSADA, ES DECIR SI QUIERES PONER UNA MATRIZ DIRECTAMENTE, LA DECLARAS TRASPOSADA
		glm::mat3 angularMatrix{ 0, angularVelocity.z, -angularVelocity.y,
			-angularVelocity.z, 0,angularVelocity.x,
			angularVelocity.y,-angularVelocity.x,0 };

		glm::quat angularSpeed4(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
		quaternion = quaternion + (angularSpeed4 * quaternion*0.5f)*dt;
		quaternion = glm::normalize(quaternion);
		rotation = glm::mat4_cast(quaternion);
	}

	void simulateEulerStep(float dt,int planoId,int vertId) {
		float newDT = dt/2;
		float trackDT = dt / 2;
		glm::vec3 linealMomL;
		glm::vec3 angMomL;
		glm::mat3 inverseInertiaL = inverseInertia;
		glm::quat quaternionL = quaternion;
		glm::mat4 rotationL = rotation;
		glm::vec3 posL;
		for (int i = 0; i < 5; i++) {
			trackDT = trackDT / 2;
			linealMomL = linealMom;
			linealMomL = linealMomL + newDT*(gravity*mass);
			angMomL = angMom + newDT*torque;
				glm::vec3 velocity = linealMomL / mass;
				posL = pos + velocity*newDT;
				inverseInertiaL = rotation * glm::inverse(iBody)*glm::transpose(rotation);
				glm::vec3 angularVelocity = inverseInertia * glm::vec3(angMom.x, angMom.y, angMom.z);
				//AL DECLARAR MATRICES EN GLM CON EL CONSTRUCTOR, RECIBE LA TRASPOSADA, ES DECIR SI QUIERES PONER UNA MATRIZ DIRECTAMENTE, LA DECLARAS TRASPOSADA
				glm::mat3 angularMatrix{ 0, angularVelocity.z, -angularVelocity.y,
					-angularVelocity.z, 0,angularVelocity.x,
					angularVelocity.y,-angularVelocity.x,0 };

				glm::quat angularSpeed4(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
				quaternionL = quaternion + (angularSpeed4 * quaternion*0.5f)*newDT;
				quaternionL = glm::normalize(quaternionL);
				rotationL = glm::mat4_cast(quaternionL);
			
				glm::vec3 verticeLocal = glm::vec3(vertices[vertId*3], vertices[vertId*3 + 1], vertices[vertId*3 + 2]);

				glm::mat4 modelMatL;

				modelMatL = glm::translate(modelMatL, posL);

				glm::mat4 tempRot;

				tempRot = rotationL;

				modelMatL *= tempRot;

				modelMatL = glm::scale(modelMatL, glm::vec3(escalado, escalado, escalado));

				verticeLocal = modelMatL * glm::vec4(verticeLocal,1);

				if ((glm::dot(planos[planoId].n, verticesAnteriores[vertId]) + planos[planoId].d)*(glm::dot(planos[planoId].n, verticeLocal) + planos[planoId].d) <= 0) {
					newDT -= trackDT;
					//std::cout << "resta\n";
				}
				else {
					newDT += trackDT;
					//std::cout << "suma\n";
				}
		}

		pos = posL;
		angMom = angMomL;
		linealMom = linealMomL;
		quaternion = quaternionL;
		rotation = rotationL;
		inverseInertia = inverseInertiaL;

		Cube::modelMat = glm::translate(Cube::modelMat, posL);

		glm::mat4 tempRot;

		tempRot = rotationL;

		Cube::modelMat *= tempRot;

		Cube::modelMat = glm::scale(Cube::modelMat, glm::vec3(escalado, escalado, escalado));



		

		UpdateDraw();
		//return modelMat;
	}

};



BoxObj 	box = BoxObj(glm::vec3(0, 5, 0),2);

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::SliderFloat("coeficienteElastico", &epsilon, 0, 1);
		ImGui::Checkbox("Colisiones",&collisions);
		restart = ImGui::Button("Restart");
		slowMo = ImGui::Button("Slow Motion");
		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
	collisions = true;
	epsilon = 1;
	//TODO
}
void PhysicsUpdate(float dt) {
	if (!start) {
		box.Restart(dt);
		start = true;
	}
	if (restart) {
		restart = false;
		box.Restart(dt);
	}

	box.Update(dt);
	Cube::drawCube();



	//TODO
}
void PhysicsCleanup() {
	//TODO
}