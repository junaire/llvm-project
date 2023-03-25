//===--- __clang_interpreter_runtime_printvalue.h - Incremental Compiation and
// Execution---*- C++
//-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines runtime functions used to print STL components in
// clang-repl. They are very heavy so we should only include it once and on
// demand.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_CLANG_INTERPRETER_RUNTIME_PRINT_VALUE_H
#define LLVM_CLANG_INTERPRETER_RUNTIME_PRINT_VALUE_H

#if !defined(__CLANG_REPL__)
#error "This file should only be included by clang-repl!"
#endif

#include <memory>
#include <string>
#include <tuple>
#include <type_traits>

// We should include it somewhere instead of duplicating it...
#if __has_attribute(visibility) &&                                             \
    (!(defined(_WIN32) || defined(__CYGWIN__)) ||                              \
     (defined(__MINGW32__) && defined(__clang__)))
#if defined(LLVM_BUILD_LLVM_DYLIB) || defined(LLVM_BUILD_SHARED_LIBS)
#define REPL_EXTERNAL_VISIBILITY __attribute__((visibility("default")))
#else
#define REPL_EXTERNAL_VISIBILITY
#endif
#else
#if defined(_WIN32)
#define REPL_EXTERNAL_VISIBILITY __declspec(dllexport)
#endif
#endif

// Fallback for e.g. vector<bool>'s bit iterator:
template <class T, class = typename std::enable_if_t<!std::is_pointer_v<T>>>
inline std::string PrintValueRuntime(const T &Val) {
  return "{not representable}";
}

// Below overloads are all defined in the library itself.
REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const void *Ptr);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const void **Ptr);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const bool *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const signed char *Val);

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned char *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const short *Val);

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned short *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const int *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const unsigned int *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long long *Val);

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned long *Val);

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned long long *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const float *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const double *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long double *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char *const *Val);

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char **Val);

namespace repl_runtime_detail {

// Avoid using C++17 std::void_t because clang-repl can be used with lower
// standards.
template <typename... T> struct repl_make_void { typedef void type; };

template <typename... T>
using repl_void_t = typename repl_make_void<T...>::type;

template <typename T, typename = void> struct is_iterable : std::false_type {};

// this gets used only when we can call std::begin() and std::end() on that type
template <typename T>
struct is_iterable<T, repl_void_t<decltype(std::begin(std::declval<T &>())),
                                  decltype(std::end(std::declval<T &>()))>>
    : std::true_type {};

template <typename> struct is_pair : std::false_type {};

template <typename T, typename U>
struct is_pair<std::pair<T, U>> : std::true_type {};

template <typename T, typename = void> struct is_map : std::false_type {};

template <typename T>
struct is_map<T, repl_void_t<typename T::mapped_type>> : std::true_type {};

// The type of the elements is std::pair, and the container is a map like type.
template <
    typename Container, typename Elt,
    typename std::enable_if<is_pair<Elt>::value && is_map<Container>::value,
                            bool>::type = true>
std::string PrintCollectionElt(const Elt &Val) {
  return PrintValueRuntime(&Val.first) + " => " +
         PrintValueRuntime(&Val.second);
}

// The type of the elements is std::pair, and the container isn't a map like
// type.
template <
    typename Container, typename Elt,
    typename std::enable_if<is_pair<Elt>::value && !is_map<Container>::value,
                            bool>::type = true>
std::string PrintCollectionElt(const Elt &Val) {
  return TuplePairPrintValue(&Val);
}

template <typename Container, typename Elt,
          typename std::enable_if<!is_pair<Elt>::value, bool>::type = true>
std::string PrintCollectionElt(const Elt &Val) {
  return PrintValueRuntime(&Val);
}

template <class Tuple, std::size_t N = std::tuple_size<Tuple>(),
          std::size_t TupleSize = std::tuple_size<Tuple>()>
struct TupleLikePrinter {
  static std::string print(const Tuple *T) {
    constexpr std::size_t EltNum = TupleSize - N;
    std::string Str;
    // Not the first element.
    if (EltNum != 0)
      Str += ", ";
    Str += PrintValueRuntime(&std::get<EltNum>(*T));
    // If N+1 is not smaller than the size of the tuple,
    // reroute the call to the printing function to the
    // no-op specialisation to stop recursion.
    constexpr std::size_t Nm1 = N - 1;
    Str += TupleLikePrinter<Tuple, Nm1>::print((const Tuple *)T);
    return Str;
  }
};

template <class Tuple, std::size_t TupleSize>
struct TupleLikePrinter<Tuple, 0, TupleSize> {
  static std::string print(const Tuple *T) { return ""; }
};

template <class T> inline std::string TuplePairPrintValue(const T *Val) {
  std::string Str("{ ");
  Str += TupleLikePrinter<T>::print(Val);
  Str += " }";
  return Str;
}

} // namespace repl_runtime_detail

template <
    typename Container,
    typename std::enable_if<repl_runtime_detail::is_iterable<Container>::value,
                            bool>::type = true>
inline std::string PrintValueRuntime(const Container *C) {
  std::string Str("{ ");

  for (auto Beg = C->begin(), End = C->end(); Beg != End; Beg++) {
    if (Beg != C->begin())
      Str += ", ";
    Str += repl_runtime_detail::PrintCollectionElt<Container>(*Beg);
  }
  Str += " }";
  return Str;
}

template <typename T, size_t N>
inline std::string PrintValueRuntime(const T (*Obj)[N]) {
  if (N == 0)
    return "{}";

  std::string Str = "{ ";
  for (size_t Idx = 0; Idx < N; ++Idx) {
    Str += PrintValueRuntime(*Obj + Idx);
    if (Idx < N - 1)
      Str += ", ";
  }
  return Str + " }";
}

template <size_t N> inline std::string PrintValueRuntime(const char (*Obj)[N]) {
  const auto *Str = reinterpret_cast<const char *const>(Obj);
  return PrintValueRuntime(&Str);
}

// tuple
template <typename... Ts>
inline std::string PrintValueRuntime(const std::tuple<Ts...> *Val) {
  using T = std::tuple<Ts...>;
  return repl_runtime_detail::TuplePairPrintValue<T>(Val);
}

// pair
template <typename... Ts>
inline std::string PrintValueRuntime(const std::pair<Ts...> *Val) {
  using T = std::pair<Ts...>;
  return repl_runtime_detail::TuplePairPrintValue<T>(Val);
}

// unique_ptr
template <class T>
inline std::string PrintValueRuntime(const std::unique_ptr<T> *Val) {
  auto Ptr = Val->get();
  return "std::unique_ptr -> " + PrintValueRuntime((const void **)&Ptr);
}

// shared_ptr
template <class T>
inline std::string PrintValueRuntime(const std::shared_ptr<T> *Val) {
  auto Ptr = Val->get();
  return "std::shared_ptr -> " + PrintValueRuntime((const void **)&Ptr);
}

// weak_ptr
template <class T>
inline std::string PrintValueRuntime(const std::weak_ptr<T> *Val) {
  auto Ptr = Val->lock().get();
  return "std::weak_ptr -> " + PrintValueRuntime((const void **)&Ptr);
}

// string
template <class T>
inline std::string PrintValueRuntime(const std::basic_string<T> *Val) {
  const char *Chars = Val->c_str();
  return PrintValueRuntime((const char **)&Chars);
}
#endif
