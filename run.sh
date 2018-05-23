echo "Compiling..."
g++ -std=c++14 -O2 -pthread convex_hull.cc -o ch

for file in `find tests -name "*.in" | cat | sort`; do
  echo "Running on" $file
  ./ch $file 8
  ./ch $file 100
  printf "\n"
done

rm ch
