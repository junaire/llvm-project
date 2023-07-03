#include <cstdio>
#include <optional>

std::optional<int> foo(int i) {
  if (i >= 0)
    return 42;
  return std::nullopt;
}

std::optional<int> bar(int i) {
  auto x = foo(i);? // Rust-like question operator in C++!!!
  return x.value() + 1;
}

void test(int i) {
  if (auto y = bar(i))
    printf("bar(%d) = %d\n", i, y.value());
  else
    printf("bar(%d) is invalid\n", i);
}
int main() {
  test(1);
  test(-1);
}
