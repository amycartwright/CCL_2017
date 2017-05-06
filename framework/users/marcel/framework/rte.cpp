#include "Path.h"
#include "internal.h"
#include "rte.h"
#include "StringEx.h"
#include <sys/stat.h>

#if defined(WIN32)
    #include <Windows.h>
#endif

static void handleFileChange(const std::string & filename)
{
	const std::string extension = Path::GetExtension(filename);

	if (extension == "vs")
	{
		for (auto i : g_shaderCache.m_map)
		{
			ShaderCacheElem & elem = i.second;

			if (elem.vs == filename)
				elem.reload();
		}
	}
	else if (extension == "ps")
	{
		for (auto i : g_shaderCache.m_map)
		{
			ShaderCacheElem & elem = i.second;

			if (elem.ps == filename)
				elem.reload();
		}
	}
	else if (extension == "cs")
	{
		for (auto i : g_computeShaderCache.m_map)
		{
			ComputeShaderCacheElem & elem = i.second;

			if (elem.name == filename)
				elem.reload();
		}
	}
	else if (extension == "inc")
	{
		clearCaches(CACHE_SHADER);
	}
	else if (extension == "png" || extension == "jpg")
	{
		Sprite(filename.c_str()).reload();
	}
	
	// call real time editing callback
	
	if (framework.realTimeEditCallback)
	{
		framework.realTimeEditCallback(filename);
	}
}

#if 1

struct FileInfo
{
	std::string filename;
	time_t time;
};

static std::vector<FileInfo> s_fileInfos;

static void fillFileInfos()
{
	s_fileInfos.clear();

	std::vector<std::string> files = listFiles(".", true);

	for (auto & file : files)
	{
		FILE * f = fopen(file.c_str(), "rb");
		if (f)
		{
			struct stat s;
			if (fstat(fileno(f), &s) == 0)
			{
				FileInfo fi;
				fi.filename = file;
				fi.time = s.st_mtime;

				if (String::EndsWith(file, ".vs") || String::EndsWith(file, ".ps") || String::EndsWith(file, ".cs") || String::EndsWith(file, ".xml") || String::EndsWith(file, ".png") || String::EndsWith(file, ".jpg"))
					s_fileInfos.push_back(fi);
			}

			fclose(f);
			f = 0;
		}
	}
}

static void clearFileInfos()
{
	s_fileInfos.clear();
}

static void checkFileInfos()
{
	for (auto & fi: s_fileInfos)
	{
		FILE * f = fopen(fi.filename.c_str(), "rb");

		if (f)
		{
			bool changed = false;

			struct stat s;
			if (fstat(fileno(f), &s) == 0)
			{
				if (fi.time < s.st_mtime)
				{
					// file has changed!

					logDebug("%s has changed!", fi.filename.c_str());

					fi.time = s.st_mtime;

					changed = true;
				}
			}

			fclose(f);
			f = 0;

			if (changed)
			{
				handleFileChange(fi.filename);
			}
		}
	}
}

#endif

#if defined(WIN32)

static HANDLE s_fileWatcher = INVALID_HANDLE_VALUE;

void initRealTimeEditing()
{
	if (s_fileWatcher != INVALID_HANDLE_VALUE)
	{
		BOOL result = FindCloseChangeNotification(s_fileWatcher);
		Assert(result);

		s_fileWatcher = INVALID_HANDLE_VALUE;
	}
	
	fillFileInfos();
	
	Assert(s_fileWatcher == INVALID_HANDLE_VALUE);
	s_fileWatcher = FindFirstChangeNotificationA(".", TRUE, FILE_NOTIFY_CHANGE_LAST_WRITE);
	Assert(s_fileWatcher != INVALID_HANDLE_VALUE);
	if (s_fileWatcher == INVALID_HANDLE_VALUE)
		logError("failed to find first change notification");
}

void shutRealTimeEditing()
{
	clearFileInfos();
}

void tickRealTimeEditing()
{
	if (s_fileWatcher != INVALID_HANDLE_VALUE)
	{
		if (WaitForSingleObject(s_fileWatcher, 0) != WAIT_OBJECT_0)
		{
			return;
		}

		Sleep(100);
	}
	
	checkFileInfos();
	
	if (s_fileWatcher != INVALID_HANDLE_VALUE)
	{
		BOOL result = FindNextChangeNotification(s_fileWatcher);
		Assert(result);

		if (!result)
		{
			logError("failed to watch for next file change notification");
		}
	}
}

#else

void initRealTimeEditing()
{
	fillFileInfos();
}

void shutRealTimeEditing()
{
	clearFileInfos();
}

void tickRealTimeEditing()
{
	// todo : add something similar to watch api on platforms other than win32
	
	static int x = 0;
	x++;
	
	if ((x % 60) != 0)
		return;
	
	checkFileInfos();
}

#endif
