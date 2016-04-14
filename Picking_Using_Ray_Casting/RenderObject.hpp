#ifndef RENDER_OBJECT_H
#define RENDER_OBJECT_H

#include <GL/glew.h>
#include <glm/glm.hpp>

class RenderObject 
{
	public:
		GLuint ID;
		glm::mat4 model;
		
		RenderObject(){}
		RenderObject(GLuint ID)
		{
			this->ID = ID;
		}

		void BindMesh_p3_c4(GLfloat vertices[], GLsizeiptr verticesLength, GLuint indices[], GLsizeiptr indicesLength) 
		{
			GLuint VBO, EBO;

			glGenVertexArrays(1, &VAO);
			glGenBuffers(1, &VBO);
			glGenBuffers(1, &EBO);

			glBindVertexArray(VAO);

			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferData(GL_ARRAY_BUFFER, verticesLength, vertices, GL_STATIC_DRAW);
			
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesLength, indices, GL_STATIC_DRAW);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)0);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(GLfloat), (GLvoid*)(3 * sizeof(GLfloat)));

			glBindVertexArray(0);

			indexCount = indicesLength / sizeof(GLuint);
		}

		void Draw_i()
		{
			glBindVertexArray(VAO);
			glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
		}

		void SetModel(glm::mat4& model)
		{
			this->model = model;
		}

	private:
		GLuint VAO;
		GLuint indexCount;
};


#endif // RENDER_OBJECT_H

