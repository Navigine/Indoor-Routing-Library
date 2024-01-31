ifndef NAVIGINE_BUILD_JOBS_NUMBER
	NAVIGINE_BUILD_JOBS_NUMBER = 1
endif

build:
	nice -10 cmake -B build -H. -DBUILD_ROUTE_GRAPH_TESTS=ON
	nice -10 cmake --build build -- -j${NAVIGINE_BUILD_JOBS_NUMBER}

tests:
	nice -10 ctest --test-dir build --output-on-failure

clean:
	rm -rf build

.PHONY : build
.PHONY : tests
