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
}

bool start = false;
bool restart = true;
bool show_test_window = false;


glm::vec3 gravity = glm::vec3(0,-9.8,0);

class BoxObj {
public:
	//Constantes
	glm::mat3 iBody;
	float mass;

	//Variables
	glm::vec3 orgPos;
	glm::vec3 pos;
	glm::mat3 rotation;
	glm::vec3 linealMom;
	glm::vec3 angMom;
	glm::vec3 torque;
	int escalado;

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

		std::cout << tempForce.x << " " << tempForce.y << " " << tempForce.z << "\n";

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
	}

	//ACTUALIZACIÓN DEL PINTADO
	void UpdateDraw() {
		Cube::modelMat = glm::mat4(1.f);
		Cube::modelMat = glm::translate(Cube::modelMat, pos);
	/*	glm::mat4 temp;

		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 2; j++) {
				temp[i][j] = rotation[i][j];
			}
		}

		Cube::modelMat *= temp;
*/
		Cube::modelMat = glm::scale(Cube::modelMat, glm::vec3(escalado, escalado, escalado));
	}

	//ACTUALIZADO
	void Update(float dt) {
		
		linealMom = linealMom + dt*(gravity*mass);
		angMom = angMom + dt*torque;

		glm::vec3 velocity = linealMom/mass;
		pos = pos + velocity*dt;

			//std::cout << iBody[0][0] << " " << iBody[0][1]<<" "<< iBody[0][2] << "\n";
			//std::cout << iBody[1][0] << " " << iBody[1][1] << " " << iBody[1][2]<< "\n";
			//std::cout << iBody[2][0] << " " << iBody[2][1] << " " << iBody[2][2] << "\n";


		//std::cout << glm::inverse(iBody)[0][0] << " " << glm::inverse(iBody)[0][1]<<" "<< glm::inverse(iBody)[0][2] << "\n";
		//std::cout << glm::inverse(iBody)[1][0] << " " << glm::inverse(iBody)[1][1] << " " << glm::inverse(iBody)[1][2]<< "\n";
		//std::cout << glm::inverse(iBody)[2][0] << " " << glm::inverse(iBody)[2][1] << " " << glm::inverse(iBody)[2][2] << "\n";


		std::cout << rotation[0][0] << " " << rotation[0][1]<<" "<< rotation[0][2] << "\n";
		std::cout << rotation[1][0] << " " << rotation[1][1] << " " << rotation[1][2]<< "\n";
		std::cout << rotation[2][0] << " " << rotation[2][1] << " " << rotation[2][2] << "\n";

			glm::mat3 inverseInertia = rotation * glm::inverse(iBody)*glm::transpose(rotation);

		//std::cout << inverseInertia[0][0] << " " << inverseInertia[0][1]<<" "<< inverseInertia[0][2] << "\n";
		//std::cout << inverseInertia[1][0] << " " << inverseInertia[1][1] << " " << inverseInertia[1][2]<< "\n";
		//std::cout << inverseInertia[2][0] << " " << inverseInertia[2][1] << " " << inverseInertia[2][2] << "\n";


		glm::vec3 angularVelocity = inverseInertia * glm::vec3(angMom.x, angMom.y, angMom.z);

		//std::cout << angularVelocity.x << " " << angularVelocity.y << " " << angularVelocity.z << std::endl;

		//AL DECLARAR MATRICES EN GLM CON EL CONSTRUCTOR, RECIBE LA TRASPOSADA, ES DECIR SI QUIERES PONER UNA MATRIZ DIRECTAMENTE, LA DECLARAS TRASPOSADA
		glm::mat3 angularMatrix{0, angularVelocity.z, -angularVelocity.y,
								-angularVelocity.z, 0,angularVelocity.x,
								angularVelocity.y,-angularVelocity.x,0 };

		rotation = rotation + dt*(angularMatrix*rotation);

		//std::cout << angularMatrix[0][0] << " " << angularMatrix[0][1] << " " << angularMatrix[0][2] << "\n";
		//std::cout << angularMatrix[1][0] << " " << angularMatrix[1][1] << " " << angularMatrix[1][2] << "\n";
		//std::cout << angularMatrix[2][0] << " " << angularMatrix[2][1] << " " << angularMatrix[2][2] << "\n";

		//std::cout << rotation[0][0] << " " << rotation[0][1]<<" "<< rotation[0][2] << "\n";
		//std::cout << rotation[1][0] << " " << rotation[1][1] << " " << rotation[1][2]<< "\n";
		//std::cout << rotation[2][0] << " " << rotation[2][1] << " " << rotation[2][2] << "\n";

		UpdateDraw();
	}
};



BoxObj 	box = BoxObj(glm::vec3(0, 1, 0),2);

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		restart = ImGui::Button("Restart");
		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
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