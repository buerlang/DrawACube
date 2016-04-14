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
GLuint FBO;

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
bool displayHidenFrameBuffer = false;

GLuint colorMod = 1;

void ID2Color(GLuint ID, GLfloat color[3]);
void Color2ID(GLubyte color[3], GLuint *ID);

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


	// create our cubes
	cube1 = RenderObject(1);
	cube1.BindMesh_p3_c4(vertices, sizeof(vertices), indices, sizeof(indices));
	cube2 = RenderObject(2);
	cube2.BindMesh_p3_c4(vertices, sizeof(vertices), indices, sizeof(indices));

	glGenFramebuffers(1, &FBO);
	glBindFramebuffer(GL_FRAMEBUFFER, FBO);
	
	// Create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
	GLuint rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, screenWidth, screenHeight); // Use a single renderbuffer object for both a depth AND stencil buffer.
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); // Now actually attach it
				
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, screenWidth, screenHeight);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbo);
	
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	projection = glm::perspective(45.0f, (GLfloat)screenWidth / (GLfloat)screenHeight, 0.1f, 100.0f);

	cube1.SetModel(glm::translate(glm::mat4(), glm::vec3(0.3f, 0.0f, -2.0f)));
	cube2.SetModel(glm::translate(glm::mat4(), glm::vec3(-0.3f, 0.0f, 0.0f)));


	return 1;
}

// Render in the other Frame Buffer
void RenderSingleColor(bool offScreen)
{
	if (!offScreen)
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	else
		glBindFramebuffer(GL_FRAMEBUFFER, FBO);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(pickProgram);

	glUniformMatrix4fv(glGetUniformLocation(pickProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(pickProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

	GLfloat pickColor[3];

	// draw the behind cube 
	glUniformMatrix4fv(glGetUniformLocation(pickProgram, "model"), 1, GL_FALSE, glm::value_ptr(cube1.model));
	ID2Color(cube1.ID, pickColor);
	glUniform3fv(glGetUniformLocation(pickProgram, "pickColor"), 1, pickColor);
	cube1.Draw_i();

	// draw the front cube 
	glUniformMatrix4fv(glGetUniformLocation(pickProgram, "model"), 1, GL_FALSE, glm::value_ptr(cube2.model));
	ID2Color(cube2.ID, pickColor);
	glUniform3fv(glGetUniformLocation(pickProgram, "pickColor"), 1, pickColor);
	cube2.Draw_i();
}

// Called everytime in the Main Loop
void onDisplay()
{
	if (rotateView)
	{
		cameraPosition = glm::vec3(glm::rotate(glm::mat4(), -((GLint)mouseX - (GLint)lastMouseX) * 2.0f / screenWidth, cameraUp) * glm::vec4(cameraPositionSaved, 1.0f));
	}
	view = glm::lookAt(cameraPosition, cameraTarget, cameraUp);

	if (displayHidenFrameBuffer) 
	{
		RenderSingleColor(false);
		return;
	}
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

// Picking Using Color Buffer 
// from the other hiden Frame Buffer
// Called when mouse clicked
void DoPick()
{
	RenderSingleColor(true);

	glFlush();
	glFinish();
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	GLubyte data[3];
	glReadPixels(mouseX, screenHeight - mouseY, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, data);
	GLuint ID;
	Color2ID(data, &ID);
	if (ID == 1 || ID == 2)
	{
		printf("Now Picked: Cube %d\n", ID);
	}
	else
	{
		printf("Now Picked: None\n");
	}
}

void ID2Color(GLuint ID, GLfloat color[3])
{
	color[0] = ((ID & 0x00ff0000) >> 16) / 255.0f * colorMod;
	color[1] = ((ID & 0x0000ff00) >> 8) / 255.0f * colorMod;
	color[2] = ((ID & 0x000000ff) >> 0) / 255.0f * colorMod;
}

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
	if (key == GLFW_KEY_UP && action == GLFW_PRESS)
		colorMod += 10;
	if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
		colorMod -= 10;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}

/* Callback Function: get the mouse position, called when mouse moved */
static void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
	mouseX = (int)xpos;
	mouseY = (int)ypos;
}

/* Callback Function: called when mouse clicked */
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
	{
		displayHidenFrameBuffer = !displayHidenFrameBuffer;
		colorMod = 1;
	}
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
