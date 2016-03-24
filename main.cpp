#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

GLuint program;
GLint attribute_coord2d;

const char *vs_source =
"#version 410\n"
"layout (location = 0) in vec3 vp;"
"layout(location = 1) in vec3 color;"
"out vec3 vColor;"
"uniform mat4 model;"
"uniform mat4 view; "
"uniform mat4 projection;"
"void main(void){"
"gl_Position = projection * view * model * vec4(vp, 1.0);"
"vColor = color;"
"}";

const char *fs_cource =
"#version 410\n"
"in vec3 vColor;"
"out vec4 outputColor;"
"void main(void){"
"outputColor = vec4(vColor, 1.0);"
"}";


int init_resources()
{
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

	return 1;
}

void onDisplay()
{
	GLfloat vertices[] = {
		-0.5f,	-0.5f,	-0.5f,	1.0f,	0.0f,	0.0f,		//0            
		0.5f,	-0.5f,	-0.5f,	1.0f,	1.0f,	0.0f,		//1            
		0.5f,	-0.5f,	0.5f,	0.0f,	1.0f,	1.0f,		//2            7-------6 
		-0.5f,	-0.5f,	0.5f,	0.0f,	1.0f,	0.0f,		//3            |\      |\	
		-0.5f,	0.5f,	-0.5f,	1.0f,	0.0f,	1.0f,		//4            | 4-----|-5
		0.5f,	0.5f,	-0.5f,	0.0f,	0.0f,	1.0f,		//5            3-|-----2 |
		0.5f,	0.5f,	0.5f,	0.4f,	0.2f,	0.8f,		//6             \|      \|
		-0.5f,	0.5f,	0.5f,	0.8f,	0.4f,	0.0f		//7              0-------1
	};
	
	GLuint indices[] = {
		0, 1, 4, 1, 5, 4,
		1, 2, 5, 2, 6, 5,
		2, 3, 6, 3, 7, 6,
		3, 0, 7, 0, 4, 7,
		0, 1, 2, 2, 3, 0,
		4, 5, 6, 6, 7, 4
	};

	GLuint VBO, VAO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);

	glm::mat4 model;
	glm::mat4 view;
	glm::mat4 projection;

	model = glm::rotate(model, 180.0f, glm::vec3(0.5f, 1.0f, 0.0f));
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));
	projection = glm::perspective(45.0f, (GLfloat)640 / (GLfloat)480, 0.1f, 100.0f);

	GLint modelLoc = glGetUniformLocation(program, "model");
	GLint viewLoc = glGetUniformLocation(program, "view");
	GLint projLoc = glGetUniformLocation(program, "projection");	

	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glBindVertexArray(VAO);
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
