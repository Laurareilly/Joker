/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	File loader utility by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_FILE_H
#define __EGPFW_UTILS_FILE_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


	// structure to hold file contents and length
	struct egpFileInfo
	{
		const char *contents;
		unsigned int length;
	};

#ifndef __cplusplus
	typedef struct egpFileInfo	egpFileInfo;
#endif	// __cplusplus


	// load file contents into a C string, return structure with 
	//	the loaded file contents and length of the string
	// if successful, output will contain valid C string pointer 
	//	and integer count, otherwise all zeros
	// 'path' param cannot be null or an empty string
	egpFileInfo egpLoadFileContents(const char *path);

	// free string
	// 'file' param cannot be null, represents file to be freed
	// returns 1 if success, 0 if not
	int egpReleaseFileContents(egpFileInfo *file);

	// utility to write a file given a raw string and length
	// returns number of bytes written if successfulm 0 if failed
	// 'path' param cannot be null or an empty string
	// 'str' param cannot be null
	// 'length' param must be greater than zero
	int egpWriteFileContents(const char *path, const char *str, const unsigned int length);


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_FILE_H