#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <netinet/in.h>
//#include <Winsock2.h>

#if defined(_MSC_VER)
#include <tchar.h>
#include <strsafe.h>
//#include <windows.h>
//#pragma comment(lib, "Ws2_32.lib")
#elif defined(__GNUC__) || defined(__GNUG__)
#include <dirent.h>
#endif

using namespace std;

int main(int argc, char* argv[]) {
	string dirname;
    if (argc != 2) {
        std::cout << "Usage is:" << endl;
		std::cout << argv[0] << " <dir name>" << std::endl;
        std::cin.get();
        exit(0);
    } else {
		dirname = argv[1];
    }

	ofstream posfile ("positives.txt");
    if (!posfile.is_open())
    {
		cout << "Unable to open file" << endl;
        return EXIT_FAILURE;
	}

#if defined(_MSC_VER)

	WIN32_FIND_DATA ffd;
	LARGE_INTEGER filesize;
	TCHAR szDir[MAX_PATH];
	size_t length_of_arg;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWORD dwError = 0;

	// Check that the input path plus 3 is not longer than MAX_PATH.
	// Three characters are for the "\*" plus NULL appended below.

	StringCchLength(dirname.c_str(), MAX_PATH, &length_of_arg);

	if (length_of_arg > (MAX_PATH - 3))
	{
		_tprintf(TEXT("\nDirectory path is too long.\n"));
		exit(EXIT_FAILURE);
	}

	_tprintf(TEXT("\nTarget directory is %s\n\n"), dirname.c_str());

	// Prepare string for use with FindFile functions.  First, copy the
	// string to a buffer, then append '\*' to the directory name.

	StringCchCopy(szDir, MAX_PATH, dirname.c_str());
	StringCchCat(szDir, MAX_PATH, TEXT("\\*"));

	// Find the first file in the directory.

	hFind = FindFirstFile(szDir, &ffd);

	if (INVALID_HANDLE_VALUE == hFind)
	{
		exit(EXIT_FAILURE);
	}

	// List all the files in the directory with some info about them.
	string img_filename;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			continue;
		}
		else
		{
			PTSTR pszFileName = ffd.cFileName;
			std::string name(pszFileName);


			img_filename = dirname + "\\" + name;
			posfile << img_filename;

			std::ifstream in(img_filename.c_str());
			unsigned int width, height;

			in.seekg(16);
			in.read((char *)&width, 4);
			in.read((char *)&height, 4);

			width = ntohl(width);
			height = ntohl(height);

			std::cout << img_filename << " (" << width << ", " << height << ")" << endl;

			posfile << " 1 0 0 " << width << " " << height << endl;


		}
	} while (FindNextFile(hFind, &ffd) != 0);

	dwError = GetLastError();
	if (dwError != ERROR_NO_MORE_FILES)
	{
		exit(EXIT_FAILURE);
	}

	FindClose(hFind);

#elif defined(__GNUC__) || defined(__GNUG__)

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (dirname.c_str())) != NULL) {
        string img_filename;
        while ((ent = readdir (dir)) != NULL) {
            //printf ("%s\n", ent->d_name);
			img_filename = dirname + "/" + ent->d_name;
			posfile << img_filename;

			std::ifstream in(img_filename.c_str());
            unsigned int width, height;

            in.seekg(16);
            in.read((char *)&width, 4);
            in.read((char *)&height, 4);

            width = ntohl(width);
            height = ntohl(height);

            std::cout << img_filename << " (" << width << ", " << height << ")" << endl;

			posfile << " 1 0 0 " << width << " " << height << endl;
        }
        closedir (dir);
    }
	else {
        /* could not open directory */
        perror ("");
        return EXIT_FAILURE;
    }

#endif

	posfile.close();
	return EXIT_SUCCESS;
}
