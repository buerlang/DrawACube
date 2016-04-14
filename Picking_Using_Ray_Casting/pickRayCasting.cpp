#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <stdlib.h>
#include <stdio.h>
#include <map>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "RenderObject.hpp"
#include "shader.h"

GLFWwindow* window;
GLuint const screenWidth = 640, screenHeight = 480;

GLuint program;
GLuint pickProgram;

GLuint mouseX, mouseY;
GLuint lastMouseX, lastMouseY;

RenderObject cube1, cube2;

glm::vec3 cameraPosition(0.0f, 1.0f, 4.0f);
glm::vec3 cameraPositionSaved;
glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);
glm::mat4 view;
glm::mat4 projection;

bool rotateView = false;

void pickColor();
void ID2Color(GLuint ID, GLfloat color[3]);
void Color2ID(GLubyte color[3], GLuint *ID);

void ScreenPosToWorldRay(
	int mouseX, int mouseY,             // Mouse position, in pixels, from bottom-left corner of the window
	int screenWidth, int screenHeight,  // Window size, in pixels
	glm::mat4& ViewMatrix,               // Camera position and orientation
	glm::mat4& ProjectionMatrix,         // Camera parameters (ratio, field of view, near and far planes)
	glm::vec3& out_origin,              // Ouput : Origin of the ray. /!\ Starts at the near plane, so if you want the ray to start at the camera's position instead, ignore this.
	glm::vec3& out_direction            // Ouput : Direction, in world space, of the ray that goes "through" the mouse.
	)
{
	glm::vec4 lRayStart_NDC(
		((float)mouseX / (float)screenWidth - 0.5f) * 2.0f, // [0,640] -> [-1,1]
		((float)mouseY / (float)screenHeight - 0.5f) * 2.0f, // [0, 480] -> [-1,1]
		-1.0f, // The near plane maps to Z=-1 in Normalized Device Coordinates
		1.0f
		);
	glm::vec4 lRayEnd_NDC(
		((float)mouseX / (float)screenWidth - 0.5f) * 2.0f,
		((float)mouseY / (float)screenHeight - 0.5f) * 2.0f,
		1.0f,
		1.0f
		);

	// Faster way (just one inverse)
	glm::mat4 M = glm::inverse(ProjectionMatrix * ViewMatrix);
	glm::vec4 lRayStart_world = M * lRayStart_NDC; lRayStart_world/=lRayStart_world.w;
	glm::vec4 lRayEnd_world   = M * lRayEnd_NDC  ; lRayEnd_world  /=lRayEnd_world.w;


	glm::vec3 lRayDir_world(lRayEnd_world - lRayStart_world);
	lRayDir_world = glm::normalize(lRayDir_world);

	// Output
	out_origin = glm::vec3(lRayStart_world);
	out_direction = glm::normalize(lRayDir_world);
}


bool TestRayOBBIntersection(
	glm::vec3 ray_origin,        // Ray origin, in world space
	glm::vec3 ray_direction,     // Ray direction (NOT target position!), in world space. Must be normalize()'d.
	glm::vec3 aabb_min,          // Minimum X,Y,Z coords of the mesh when not transformed at all.
	glm::vec3 aabb_max,          // Maximum X,Y,Z coords. Often aabb_min*-1 if your mesh is centered, but it's not always the case.
	glm::mat4& modelMatrix,       // Transformation applied to the mesh (which will thus be also applied to its bounding box)
	float& intersection_distance // Output : distance between ray_origin and the intersection with the OBB
	) 
{
	float tMin = 0.0f;
	float tMax = 100000.0f;

	glm::vec3 OBBposition_worldspace(modelMatrix[3].x, modelMatrix[3].y, modelMatrix[3].z);

	glm::vec3 delta = OBBposition_worldspace - ray_origin;

	// Test intersection with the 2 planes perpendicular to the OBB's X axis
	{
		glm::vec3 xaxis(modelMatrix[0].x, modelMatrix[0].y, modelMatrix[0].z);
		float e = glm::dot(xaxis, delta);
		float f = glm::dot(ray_direction, xaxis);

		if (fabs(f) > 0.001f) { // Standard case

			float t1 = (e + aabb_min.x) / f; // Intersection with the "left" plane
			float t2 = (e + aabb_max.x) / f; // Intersection with the "right" plane
			if (t1>t2) {
				float w = t1; t1 = t2; t2 = w; // swap t1 and t2
			}

			if (t2 < tMax)
				tMax = t2;
			if (t1 > tMin)
				tMin = t1;

			if (tMax < tMin)
				return false;

		}
		else { // Rare case : the ray is almost parallel to the planes, so they don't have any "intersection"
			if (-e + aabb_min.x > 0.0f || -e + aabb_max.x < 0.0f)
				return false;
		}
	}


	// Test intersection with the 2 planes perpendicular to the OBB's Y axis
	// Exactly the same thing than above.
	{
		glm::vec3 yaxis(modelMatrix[1].x, modelMatrix[1].y, modelMatrix[1].z);
		float e = glm::dot(yaxis, delta);
		float f = glm::dot(ray_direction, yaxis);

		if (fabs(f) > 0.001f) {

			float t1 = (e + aabb_min.y) / f;
			float t2 = (e + aabb_max.y) / f;

			if (t1>t2) { float w = t1; t1 = t2; t2 = w; }

			if (t2 < tMax)
				tMax = t2;
			if (t1 > tMin)
				tMin = t1;
			if (tMin > tMax)
				return false;

		}
		else {
			if (-e + aabb_min.y > 0.0f || -e + aabb_max.y < 0.0f)
				return false;
		}
	}


	// Test intersection with the 2 planes perpendicular to the OBB's Z axis
	// Exactly the same thing than above.
	{
		glm::vec3 zaxis(modelMatrix[2].x, modelMatrix[2].y, modelMatrix[2].z);
		float e = glm::dot(zaxis, delta);
		float f = glm::dot(ray_direction, zaxis);

		if (fabs(f) > 0.001f) {

			float t1 = (e + aabb_min.z) / f;
			float t2 = (e + aabb_max.z) / f;

			if (t1>t2) { float w = t1; t1 = t2; t2 = w; }

			if (t2 < tMax)
				tMax = t2;
			if (t1 > tMin)
				tMin = t1;
			if (tMin > tMax)
				return false;

		}
		else {
			if (-e + aabb_min.z > 0.0f || -e + aabb_max.z < 0.0f)
				return false;
		}
	}

	intersection_distance = tMin;
	return true;

}

int init_resources()
{
	// Depth Testing 
	glEnable(GL_DEPTH_TEST);
	//glDepthFunc(GL_ALWAYS);		// ignore depth testing


	// Blending 
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	// Face Culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Load Shaders
	program = LoadShader("vertex.vert", "fragment.frag");
	pickProgram = LoadShader("pickVertex.vert", "pickFragment.frag");



	GLfloat vertices[] = {
		// vertex postion		    // vertex color
		// x        y       z       // r      g       b       a
		-0.5f,	-0.5f,	 0.5f,		0.0f,	1.0f,	0.0f,	1.0f,		//0            
		0.5f,	-0.5f,	 0.5f,		1.0f,	1.0f,	0.0f,	1.0f,		//1            
		0.5f,	-0.5f,	-0.5f,		0.0f,	1.0f,	1.0f,	1.0f,		//2            7-------6 
		-0.5f,	-0.5f,	-0.5f,		1.0f,	0.0f,	0.0f,	1.0f,		//3            |\      |\	
		-0.5f,	 0.5f,	 0.5f,		1.0f,	0.0f,	1.0f,	1.0f,		//4            | 4-----|-5
		0.5f,	 0.5f,   0.5f,		0.0f,	0.0f,	1.0f,	1.0f,		//5            3-|-----2 |
		0.5f,	 0.5f,  -0.5f,		0.4f,	0.2f,	0.8f,	1.0f,		//6             \|      \|
		-0.5f,	 0.5f,  -0.5f,		0.8f,	0.4f,	0.0f,	1.0f		//7              0-------1
	};

	GLuint indices[] = {
		0, 1, 4, 1, 5, 4,
		1, 2, 5, 2, 6, 5,
		2, 3, 6, 3, 7, 6,
		3, 0, 7, 0, 4, 7,
		1, 0, 2, 3, 2, 0,
		4, 5, 7, 5, 6, 7,
	};


	// Create our cubes
	cube1 = RenderObject(1);
	cube1.BindMesh_p3_c4(vertices, sizeof(vertices), indices, sizeof(indices));
	cube2 = RenderObject(2);
	cube2.BindMesh_p3_c4(vertices, sizeof(vertices), indices, sizeof(indices));


	// Setup Projection Matrix
	projection = glm::perspective(45.0f, (GLfloat)screenWidth / (GLfloat)screenHeight, 0.1f, 100.0f);
	

	return 1;
}

void onDisplay()
{
	// Setup View Matrix
	if (rotateView)
	{
		cameraPosition = glm::vec3(glm::rotate(glm::mat4(), -((GLint)mouseX - (GLint)lastMouseX) * 2.0f / screenWidth, cameraUp) * glm::vec4(cameraPositionSaved, 1.0f));
	}
	view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);

	// Setup each Model Matrix
	cube1.SetModel(glm::translate(glm::mat4(), glm::vec3(0.3f, 0.0f, -2.0f)));
	cube2.SetModel(glm::translate(glm::mat4(), glm::vec3(-0.3f, 0.0f, 1.0f)));


	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);

	glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

	// draw the behind cube 
	glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(cube1.model));
	cube1.Draw_i();

	// draw the front cube 
	glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(cube2.model));
	cube2.Draw_i();

}


void DoPick()
{
	glm::vec3 ray_origin;
	glm::vec3 ray_direction;
	ScreenPosToWorldRay(
		mouseX, mouseY,
		screenWidth, screenHeight,
		view,
		projection,
		ray_origin,
		ray_direction
		);

	
	float intersection_distance; // Output of TestRayOBBIntersection()
	glm::vec3 aabb_min(-0.5f, -0.5f, -0.5f);
	glm::vec3 aabb_max(0.5f, 0.5f, 0.5f);
		
	
	if (TestRayOBBIntersection(
		ray_origin,
		ray_direction,
		aabb_min,
		aabb_max,
		cube1.model,
		intersection_distance)
		) {
		printf("Now Picked: Cube 1\n");
	}
	else if (TestRayOBBIntersection(
		ray_origin,
		ray_direction,
		aabb_min,
		aabb_max,
		cube2.model,
		intersection_distance)
		) {
		printf("Now Picked: Cube 2\n");
	}
	else {
		printf("Now Picked: None\n");
	}

}

// Convert an ID to a corresponding color
void ID2Color(GLuint ID, GLfloat color[3])
{
	color[0] = ((ID & 0x00ff0000) >> 16) / 255.0f;
	color[1] = ((ID & 0x0000ff00) >> 8) / 255.0f;
	color[2] = ((ID & 0x000000ff) >> 0) / 255.0f;
}

// Convert a color back to its corresponding ID
void Color2ID(GLubyte color[3], GLuint *ID)
{
	*ID = color[0] * 256 * 256 + color[1] * 256 + color[2];
}

void free_resources()
{
	glDeleteProgram(program);
}

static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}

// Callback Function: get the mouse position, called when mouse moved
static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
	mouseX = (int)xpos;
	mouseY = screenHeight - (int)ypos;
	
}

// Callback Function: called when mouse clicked
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
	{
		DoPick();
	}
	if (button == GLFW_MOUSE_BUTTON_MIDDLE)
	{
		if (action == GLFW_PRESS) {
			rotateView = true;
			lastMouseX = mouseX;
			lastMouseY = mouseY;
			cameraPositionSaved = cameraPosition;
		}	
		else if (action == GLFW_RELEASE)
			rotateView = false;
	}
}

int main(void)
{
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(screenWidth, screenHeight, "Simple example", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);

	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();
	if (glewError != GLEW_OK)
	{
		//      throw std::runtime_error("glew fails to start.");
		fprintf(stderr, "glew error\n");
	}

	// get version info
	const GLubyte* renderer = glGetString(GL_RENDERER); // get renderer string
	const GLubyte* version = glGetString(GL_VERSION); // version as a string
	printf("Renderer: %s\n", renderer);
	printf("OpenGL version supported %s\n", version);

	// tell GL to only draw onto a pixel if the shape is closer to the viewer
	glEnable(GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc(GL_LESS); // depth-testing interprets a smaller value as "closer"

	glfwSetKeyCallback(window, key_callback);
	glfwSetCursorPosCallback(window, cursor_pos_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);

	GLint maxV;
	glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &maxV);
	fprintf(stderr, "maxv: %d\n", maxV);

	init_resources();

	while (!glfwWindowShouldClose(window))
	{

		onDisplay();
		glfwSwapBuffers(window);
		glfwPollEvents();

	}

	free_resources();

	glfwDestroyWindow(window);
	glfwTerminate();

	exit(EXIT_SUCCESS);
}
