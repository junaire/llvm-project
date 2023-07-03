template <typename T> inline bool __RustQuestionOp_HasValue(const T &ADT) {
  return !ADT.has_value();
}
template <typename T> inline T __RustQuestionOp_ReturnNull(const T &ADT) {
  return T{};
}
