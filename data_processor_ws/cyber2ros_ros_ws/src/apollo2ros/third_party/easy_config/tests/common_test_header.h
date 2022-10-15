#include <doctest/doctest.h>
#include <cstring>
#include <vector>

class dt_removed
{
	std::vector<const char*> vec;
public:
	dt_removed() = default;
	dt_removed(char** argv_in)
	{
		for (; *argv_in; ++argv_in)
			if (strncmp(*argv_in, "--dt-", strlen("--dt-")) != 0)
				vec.push_back(*argv_in);
		vec.push_back(NULL);
	}

	int          argc() { return static_cast<int>(vec.size()) - 1; }
	const char** argv() { return &vec[0]; } // Note: non-const char **:
};

extern dt_removed args;
