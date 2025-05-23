#pragma once
#include "GLUniformAbstractions.h"
#include <glad/gl.h>

template <typename T>
class Uniform
{
public:
	Uniform(GLint location) : m_location(location) {}
	void set(const T &value)
	{
		glUniform(m_location, value);
	}

private:
	GLint m_location;
};
