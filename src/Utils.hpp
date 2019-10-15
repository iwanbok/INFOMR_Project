#pragma once

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

char *getCmdOption(int argc, char **argv, const std::string &option)
{
	char **itr = std::find(argv, argv + argc, option);
	if (itr != argv + argc && ++itr != argv + argc)
	{
		return *itr;
	}
	return 0;
}

bool cmdOptionExists(int argc, char **argv, const std::string &option)
{
	return std::find(argv, argv + argc, option) != argv + argc;
}

std::filesystem::path replaceDir(std::filesystem::path old, std::string newPath)
{
    auto dir = std::filesystem::path(newPath) / old.parent_path().filename();
    if(!std::filesystem::exists(dir))
        std::filesystem::create_directories(dir);
	return dir / old.filename();
}

static bool endsWith(const std::string &str, const std::string &ending)
{
	if (str.length() < ending.length())
		return false;
	return str.compare(str.length() - ending.length(), ending.length(), ending) == 0;
}

/*
string split implementation by using delimiter as a character.
*/
std::vector<std::string> split(std::string strToSplit, char delimeter)
{
	std::stringstream ss(strToSplit);
	std::string item;
	std::vector<std::string> splittedStrings;
	while (getline(ss, item, delimeter))
	{
		splittedStrings.push_back(item);
	}
	return splittedStrings;
}

// Based on
// https://thispointer.com/c-get-the-list-of-all-files-in-a-given-directory-and-its-sub-directories-using-boost-c17/
/*
 * Get the list of all files in given directory and its sub directories.
 *
 * Arguments
 * 	dirPath : Path of directory to be traversed
 * 	dirSkipList : List of folder names to be skipped
 *
 * Returns:
 * 	vector containing paths of all the files in given directory and its sub directories
 *
 */
std::vector<std::filesystem::path> getAllFilesInDir(const std::string &dirPath,
													const std::vector<std::string> dirSkipList = {})
{
	namespace filesys = std::filesystem;
	// Create a vector of string
	std::vector<filesys::path> listOfFiles;
	try
	{
		// Check if given path exists and points to a directory
		if (filesys::exists(dirPath) && filesys::is_directory(dirPath))
		{
			// Create a Recursive Directory Iterator object and points to the starting of directory
			filesys::recursive_directory_iterator iter(dirPath);

			// Create a Recursive Directory Iterator object pointing to end.
			filesys::recursive_directory_iterator end;

			// Iterate till end
			while (iter != end)
			{
				// Check if current entry is a directory and if exists in skip list
				if (filesys::is_directory(iter->path()) &&
					(std::find(dirSkipList.begin(), dirSkipList.end(), iter->path().filename()) !=
					 dirSkipList.end()))
					// Skip the iteration of current directory pointed by iterator
					iter.disable_recursion_pending();
				else if (!filesys::is_directory(iter->path()))
					// Add the name in vector
					listOfFiles.push_back(iter->path().string());

				std::error_code ec;
				// Increment the iterator to point to next entry in recursive iteration
				iter.increment(ec);
				if (ec)
				{
					std::cerr << "Error While Accessing : " << iter->path().string()
							  << " :: " << ec.message() << std::endl;
				}
			}
		}
	}
	catch (std::system_error &e)
	{
		std::cerr << "Exception :: " << e.what();
	}
	return listOfFiles;
}