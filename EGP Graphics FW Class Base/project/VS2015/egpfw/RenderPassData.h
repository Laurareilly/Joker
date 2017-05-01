#pragma once
#include <vector>
#include "egpfw/egpfw/utils/egpfwShaderProgramUtils.h"
#include <GL/glew.h>
#include <cbmath/cbtkMatrix.h>

/**
 * \brief Template struct used for sending data that is already contiguous in memory.
 * \tparam E Our type enum (i.e. egpIntType, egpFloatType, etc.)
 * \tparam T Our actual data storage type (i.e. int, float, etc.) */
template <typename E, typename T>
struct UniformDataSimple
{
	int location;
	E type;
	unsigned int count;
	T* values;

	/**
	 * \param l Location
	 * \param t Type
	 * \param c Count
	 * \param v Value buffer */
	UniformDataSimple(int l, E t, unsigned int c, T* v) : location(l), type(t), count(c), values(v) {}
};

/**
 * \brief Template struct used for sending data that is not contiguous in memory.
 * \tparam E Our type enum (i.e. egpIntType, egpFloatType, etc.)
 * \tparam T Our actual data storage type (i.e. int, float, etc.) */
template <typename E, typename T>
struct UniformDataComplex
{
	int location;
	E type;
	unsigned int count;
	std::vector<T*> values;

	/**
	 * \param l Location
	 * \param t Type
	 * \param c Count
	 * \param v std::initializer_list of addresses where the data will be pulled and combined from. */
	UniformDataComplex(int l, E t, unsigned int c, std::initializer_list<T*> v) : location(l), type(t), count(c), values(v) {}
};

/**
 * \brief Struct used for sending matrices. */
struct UniformMatrixData
{
	int location;
	unsigned int count;
	int transpose;
	cbmath::mat4* value;

	/**
	 * \param l Location
	 * \param c Count
	 * \param tr Transpose?
	 * \param v Address of the cbmath::mat4 whose data will be pulled. */
	UniformMatrixData(int l, unsigned int c, int tr, cbmath::mat4* v) : location(l), count(c), transpose(tr), value(v) {}
};

/**
 * \brief Struct used for sending an FBO color texture target. */
struct FBOTargetColorTexture
{
	int fboIndex;
	unsigned int glBinding;
	int targetIndex;

	/**
	 * \param f FBO index in the global FBO array.
	 * \param b glBinding
	 * \param t TargetIndex */
	FBOTargetColorTexture(int f, unsigned int b, int t) : fboIndex(f), glBinding(b), targetIndex(t) {}
};

/**
 * \brief Struct used for sending an FBO depth texture target. */
struct FBOTargetDepthTexture
{
	int fboIndex;
	unsigned int glBinding;

	/**
	 * \param f FBO index in the global FBO array.
	 * \param b glBinding */
	FBOTargetDepthTexture(int f, unsigned int b) : fboIndex(f), glBinding(b) {}
};

/**
 * \brief Struct used for sending texture data. */
struct RenderPassTextureData
{
	GLenum textureType;
	GLuint textureLane;
	GLuint textureHandle;

	/**
	 * \param type What is the type of texture (GL_TEXTURE_2D, etc.)
	 * \param lane Which texture location to use (GL_TEXTURE0, etc.)
	 * \param handle Handle to the glTexture. */
	RenderPassTextureData(GLenum type, GLuint lane, GLuint handle) : textureType(type), textureLane(lane), textureHandle(handle) {}
};

//Some typedefs to make everything look cleaner.
typedef UniformDataSimple<egpUniformIntType, int> render_pass_uniform_int;
typedef UniformDataSimple<egpUniformFloatType, float> render_pass_uniform_float;
typedef UniformDataComplex<egpUniformFloatType, float> render_pass_uniform_float_complex;
typedef UniformMatrixData render_pass_uniform_float_matrix;