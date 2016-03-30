#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

GLuint program;
GLint attribute_coord2d;
GLuint VBO, VAO, EBO;

const char *vs_source =
"#version 410\n"
"layout (location = 0) in vec3 vp;"
"layout(location = 1) in vec4 color;"
"out vec4 v2fColor;"
"uniform mat4 model;"
"uniform mat4 view; "
"uniform mat4 projection;"
"void main(void){"
"gl_Position = projection * view * model * vec4(vp, 1.0);"
"v2fColor = color;"
"}";

const char *fs_cource =
"#version 410\n"
"in vec4 v2fColor;"
"out vec4 outputColor;"
"void main(void){"
"outputColor = v2fColor;"
"}";


int init_resources()
{
/* Depth Testing */
	glEnable(GL_DEPTH_TEST);
	//glDepthFunc(GL_ALWAYS);		// ignore depth testing


/* Blending */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


/* Face Culling */
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CW);


	GLint compile_ok = GL_FALSE, link_ok = GL_FALSE;

	GLuint vs = glCreateShader(GL_VERTEX_SHADER);

	glShaderSource(vs, 1, &vs_source, NULL);
	glCompileShader(vs);
	glGetShaderiv(vs, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE)
	{

		GLint infoLogLength;
		glGetShaderiv(vs, GL_INFO_LOG_LENGTH, &infoLogLength);

		GLchar *strInfoLog = new GLchar[infoLogLength + 1];
		glGetShaderInfoLog(vs, infoLogLength, NULL, strInfoLog);

		fprintf(stderr, "Compile failure in shader:\n%s\n", strInfoLog);
		delete[] strInfoLog;
		fprintf(stderr, "error in vertex error\n");
		return 0;
	}

	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(fs, 1, &fs_cource, NULL);
	glCompileShader(fs);
	glGetShaderiv(fs, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE)
	{
		GLint infoLogLength;
		glGetShaderiv(vs, GL_INFO_LOG_LENGTH, &infoLogLength);

		GLchar *strInfoLog = new GLchar[infoLogLength + 1];
		glGetShaderInfoLog(fs, infoLogLength, NULL, strInfoLog);

		fprintf(stderr, "Compile failure in shader:\n%s\n", strInfoLog);
		delete[] strInfoLog;

		fprintf(stderr, "error in fragment shader\n");
		return 0;
	}

	program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
	if (!link_ok)
	{
		fprintf(stderr, "glinkprogram\n");
		return 0;
	}

	const char* attribute_name = "vp";
	attribute_coord2d = glGetAttribLocation(program, attribute_name);
	if (attribute_coord2d == -1)
	{
		fprintf(stderr, "cannot bind attribute");
		return 0;
	}


	GLfloat vertices[] = {
	   // vertex postion		   // vertex color
	   // x        y       z       // r      g       b       a
		-0.5f,	-0.5f,	 0.5f,		0.0f,	1.0f,	0.0f,	0.5f,		//0            
		 0.5f,	-0.5f,	 0.5f,		1.0f,	1.0f,	0.0f,	0.5f,		//1            
		 0.5f,	-0.5f,	-0.5f,		0.0f,	1.0f,	1.0f,	0.5f,		//2            7-------6 
		-0.5f,	-0.5f,	-0.5f,		1.0f,	0.0f,	0.0f,	0.5f,		//3            |\      |\	
		-0.5f,	 0.5f,	 0.5f,		1.0f,	0.0f,	1.0f,	0.5f,		//4            | 4-----|-5
		 0.5f,	 0.5f,	 0.5f,		0.0f,	0.0f,	1.0f,	0.5f,		//5            3-|-----2 |
		 0.5f,	 0.5f,	-0.5f,		0.4f,	0.2f,	0.8f,	0.5f,		//6             \|      \|
		-0.5f,	 0.5f,	-0.5f,		0.8f,	0.4f,	0.0f,	0.5f		//7              0-------1
	};

	GLuint indices[] = {
		0, 1, 4,	1, 5, 4,
		1, 2, 5,	2, 6, 5,
		2, 3, 6,	3, 7, 6,
		3, 0, 7,	0, 4, 7,
		1, 0, 2,	3, 2, 0,
		4, 5, 7,	5, 6, 7
	};

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	
	glBindVertexArray(0);


	return 1;
}

void onDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);

	glm::mat4 model;
	glm::mat4 view;
	glm::mat4 projection;

	view = glm::lookAt(
		glm::vec3(0.0f, 1.0f, 2.0f),
		glm::vec3(0.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 1.0f, 0.0f)
		);
	projection = glm::perspective(45.0f, (GLfloat)640 / (GLfloat)480, 0.1f, 100.0f);

	glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

	glBindVertexArray(VAO);


/* draw the behind cube */
	model = glm::mat4();
	model = glm::translate(model, glm::vec3(0.3f, 0.0f, -2.0f));
	glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

/* draw the front cube */
	model = glm::mat4();
	model = glm::translate(model, glm::vec3(-0.3f, 0.0f, 0.0f));
	glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

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
int main(void)
{



	GLFWwindow* window;
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
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
