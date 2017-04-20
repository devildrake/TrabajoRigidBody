#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <cassert>
#include "GL_framework.h"
#include <iostream>
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

bool show_test_window = false;


glm::vec3 gravity = glm::vec3(0,-9.8,0);

class BoxObj {
public:
	//Constantes
	glm::mat3 iBody;
	float mass;

	//Variables
	glm::vec3 pos;
	glm::mat3 rotation;
	glm::vec3 linealMom;
	glm::vec3 angMom;
	
	int size;
	void UpdateDraw() {
		Cube::modelMat = glm::mat4(1.f);
		Cube::modelMat = glm::scale(Cube::modelMat, glm::vec3(size, size, size));
		Cube::modelMat = glm::translate(Cube::modelMat, pos);
	}
	BoxObj(glm::vec3 orgPos, int s){
		pos = orgPos;
		size = s;
		mass = size;
		linealMom = linealMom + (gravity*mass);
		//iBody es la misma en las tres partes de la matriz porque es un cubo y los lados son iguales
		iBody[0][0] = iBody[1][1] = iBody[2][2] = (1 / 12)*mass*(size*size + size*size);

		glm::vec3 temp = glm::vec3(rand(),rand(),rand());

	}
	void Update(float dt) {

		linealMom = linealMom + dt*(gravity*mass);
		UpdateDraw();
	}
};

BoxObj 	box = BoxObj(glm::vec3(0, 3, 0),2);

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

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
	box.Update(dt);
	Cube::drawCube();
	//TODO
}
void PhysicsCleanup() {
	//TODO
}