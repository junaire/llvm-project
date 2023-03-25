#include "InterpreterUtils.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/PrettyPrinter.h"
#include "clang/AST/Type.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Interpreter/Interpreter.h"
#include "clang/Interpreter/Value.h"
#include "clang/Parse/Parser.h"
#include "clang/Sema/Lookup.h"
#include "clang/Sema/Sema.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/raw_ostream.h"
#include <cassert>
#include <string>

using namespace clang;
using namespace clang::caas;

static std::string PrintDeclType(const QualType &QT, NamedDecl *D) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  if (QT.hasQualifiers())
    SS << QT.getQualifiers().getAsString() << " ";
  SS << D->getQualifiedNameAsString();
  return Str;
}

static std::string PrintQualType(ASTContext &Ctx, QualType QT) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  PrintingPolicy Policy(Ctx.getPrintingPolicy());
  // Print the Allocator in STL containers, for instance.
  Policy.SuppressDefaultTemplateArgs = false;
  Policy.SuppressUnwrittenScope = true;
  // Print 'a<b<c> >' rather than 'a<b<c>>'.
  Policy.SplitTemplateClosers = true;

  struct LocalPrintingPolicyRAII {
    ASTContext &Context;
    PrintingPolicy Policy;

    LocalPrintingPolicyRAII(ASTContext &Ctx, PrintingPolicy &PP)
        : Context(Ctx), Policy(Ctx.getPrintingPolicy()) {
      Context.setPrintingPolicy(PP);
    }
    ~LocalPrintingPolicyRAII() { Context.setPrintingPolicy(Policy); }
  } X(Ctx, Policy);

  const QualType NonRefTy = QT.getNonReferenceType();

  if (const auto *TTy = llvm::dyn_cast<TagType>(NonRefTy))
    SS << PrintDeclType(NonRefTy, TTy->getDecl());
  else if (const auto *TRy = dyn_cast<RecordType>(NonRefTy))
    SS << PrintDeclType(NonRefTy, TRy->getDecl());
  else {
    const QualType Canon = NonRefTy.getCanonicalType();
    if (Canon->isBuiltinType() && !NonRefTy->isFunctionPointerType() &&
        !NonRefTy->isMemberPointerType()) {
      SS << Canon.getAsString(Ctx.getPrintingPolicy());
    } else if (const auto *TDTy = dyn_cast<TypedefType>(NonRefTy)) {
      // FIXME: TemplateSpecializationType & SubstTemplateTypeParmType checks
      // are predominately to get STL containers to print nicer and might be
      // better handled in GetFullyQualifiedName.
      //
      // std::vector<Type>::iterator is a TemplateSpecializationType
      // std::vector<Type>::value_type is a SubstTemplateTypeParmType
      //
      QualType SSDesugar = TDTy->getLocallyUnqualifiedSingleStepDesugaredType();
      if (llvm::isa<SubstTemplateTypeParmType>(SSDesugar))
        SS << GetFullTypeName(Ctx, Canon);
      else if (llvm::isa<TemplateSpecializationType>(SSDesugar))
        SS << GetFullTypeName(Ctx, NonRefTy);
      else
        SS << PrintDeclType(NonRefTy, TDTy->getDecl());
    } else
      SS << GetFullTypeName(Ctx, NonRefTy);
  }

  if (QT->isReferenceType())
    SS << " &";

  return Str;
}

static std::string PrintEnum(const Value &V) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  ASTContext &Ctx = const_cast<ASTContext &>(V.getASTContext());

  QualType DesugaredTy = V.getType().getDesugaredType(Ctx);
  const EnumType *EnumTy = DesugaredTy.getNonReferenceType()->getAs<EnumType>();
  assert(EnumTy && "Fail to cast to enum type");

  EnumDecl *ED = EnumTy->getDecl();
  uint64_t Data = V.getULongLong();
  bool IsFirst = true;
  llvm::APSInt AP = Ctx.MakeIntValue(Data, DesugaredTy);

  for (auto I = ED->enumerator_begin(), E = ED->enumerator_end(); I != E; ++I) {
    if (I->getInitVal() == AP) {
      if (!IsFirst)
        SS << " ? ";
      SS << "(" + I->getQualifiedNameAsString() << ")";
      IsFirst = false;
    }
  }

  SS << " : " << PrintQualType(Ctx, ED->getIntegerType()) << " "
     << llvm::toString(AP, /*Radix=*/10);
  return Str;
}

static std::string PrintFunction(const Value &V, const void *Ptr) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << "Function @" << Ptr;

  const FunctionDecl *FD = nullptr;

  auto Decls = V.getASTContext().getTranslationUnitDecl()->decls();
  assert(std::distance(Decls.begin(), Decls.end()) == 1 &&
         "TU should only contain one Decl");
  auto *TLSD = llvm::cast<TopLevelStmtDecl>(*Decls.begin());

  // Get __clang_Interpreter_SetValueNoAlloc(void *This, void *OutVal, void
  // *OpaqueType, void *Val);
  if (auto *InterfaceCall = llvm::dyn_cast<CallExpr>(TLSD->getStmt())) {
    const auto *Arg = InterfaceCall->getArg(/*Val*/ 3);
    // Get rid of cast nodes.
    while (const CastExpr *CastE = llvm::dyn_cast<CastExpr>(Arg))
      Arg = CastE->getSubExpr();
    if (const DeclRefExpr *DeclRefExp = llvm::dyn_cast<DeclRefExpr>(Arg))
      FD = llvm::dyn_cast<FunctionDecl>(DeclRefExp->getDecl());

    if (FD) {
      SS << '\n';
      const clang::FunctionDecl *FDef;
      if (FD->hasBody(FDef))
        FDef->print(SS);
    }
  }
  return Str;
}

static std::string PrintAddress(const void *Ptr, char Prefix) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  if (!Ptr)
    return Str;
  SS << Prefix << Ptr;
  return Str;
}

// TODO: Encodings.
static std::string PrintOneChar(char Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);

  SS << "'" << Val << "'";
  return Str;
}

// Char pointers
// Assumption is this is a string.
// N is limit to prevent endless loop if Ptr is not really a string.
static std::string PrintString(const char *const *Ptr, size_t N = 10000) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);

  const char *Start = *Ptr;
  if (!Start)
    return "nullptr";

  const char *End = Start + N;
  // If we're gonnd do this, better make sure the end is valid too
  // FIXME: getpagesize() & GetSystemInfo().dwPageSize might be better
  static constexpr auto PAGE_SIZE = 1024;
  while (N > 1024) {
    N -= PAGE_SIZE;
    End = Start + N;
  }

  if (*Start == 0)
    return "\"\"";

  // Copy the bytes until we get a null-terminator
  SS << "\"";
  while (Start < End && *Start)
    SS << *Start++;
  SS << "\"";

  return Str;
}

// Build the CallExpr to `PrintValueRuntime`.
static void BuildWrapperBody(Interpreter &Interp, Sema &S, ASTContext &Ctx,
                             FunctionDecl *WrapperFD, QualType QT,
                             const void *ValPtr) {
  Sema::SynthesizedFunctionScope SemaFScope(S, WrapperFD);
  clang::Parser::ParseScope PS(
      &Interp.getParser(), clang::Scope::FnScope | clang::Scope::BlockScope);

  clang::DeclarationName RuntimeCallName =
      S.PP.getIdentifierInfo("PrintValueRuntime");
  clang::LookupResult R(S, RuntimeCallName, SourceLocation(),
                        clang::Sema::LookupOrdinaryName);
  S.LookupQualifiedName(R, Ctx.getTranslationUnitDecl());

  Expr *OverldExpr = UnresolvedLookupExpr::Create(
      Ctx, /*NamingClass=*/nullptr, NestedNameSpecifierLoc(),
      clang::DeclarationNameInfo(RuntimeCallName, SourceLocation()),
      /*RequiresADL*/ false, R.isOverloadedResult(), R.begin(), R.end());

  if (const auto *AT = llvm::dyn_cast<AutoType>(QT.getTypePtr())) {
    if (AT->isDeduced())
      QT = AT->getDeducedType().getDesugaredType(Ctx);
  }

  if (const auto *PT = llvm::dyn_cast<PointerType>(QT.getTypePtr())) {
    // Normalize `X*` to `const void*`, invoke `printValue(const void**)`,
    // unless it's a character string.
    QualType QTPointeeUnqual = PT->getPointeeType().getUnqualifiedType();
    if (!Ctx.hasSameType(QTPointeeUnqual, Ctx.CharTy) &&
        !Ctx.hasSameType(QTPointeeUnqual, Ctx.WCharTy) &&
        !Ctx.hasSameType(QTPointeeUnqual, Ctx.Char16Ty) &&
        !Ctx.hasSameType(QTPointeeUnqual, Ctx.Char32Ty)) {
      QT = Ctx.getPointerType(Ctx.VoidTy.withConst());
    }
  } else if (const auto *RTy = llvm::dyn_cast<ReferenceType>(QT.getTypePtr())) {
    // X& will be printed as X* (the pointer will be added below).
    QT = RTy->getPointeeType();
    // Val will be a X**, but we cast this to X*, so dereference here:
    ValPtr = *(const void *const *)ValPtr;
  }

  // `PrintValueRuntime()` takes the *address* of the value to be printed:
  QualType QTPtr = Ctx.getPointerType(QT);
  Expr *TypeArg = CStyleCastPtrExpr(S, QTPtr, (uintptr_t)ValPtr);
  llvm::SmallVector<Expr *, 1> CallArgs = {TypeArg};

  // Create the CallExpr.
  ExprResult RuntimeCall =
      S.ActOnCallExpr(S.getCurScope(), OverldExpr, SourceLocation(), CallArgs,
                      SourceLocation());
  assert(!RuntimeCall.isInvalid() && "Cannot create call to PrintValueRuntime");

  // Create the ReturnStmt.
  StmtResult RetStmt =
      S.ActOnReturnStmt(SourceLocation(), RuntimeCall.get(), S.getCurScope());
  assert(!RetStmt.isInvalid() && "Cannot create ReturnStmt");

  // Create the CompoundStmt.
  StmtResult Body =
      CompoundStmt::Create(Ctx, {RetStmt.get()}, FPOptionsOverride(),
                           SourceLocation(), SourceLocation());
  assert(!Body.isInvalid() && "Cannot create function body");

  WrapperFD->setBody(Body.get());
  // Add attribute `__attribute__((used))`.
  WrapperFD->addAttr(UsedAttr::CreateImplicit(Ctx));
}

static constexpr const char *const WrapperName = "__InterpreterCallPrint";

static std::string SynthesizeRuntimePrint(const Value &V) {
  Interpreter &Interp = const_cast<Interpreter &>(V.getInterpreter());
  Sema &S = Interp.getCompilerInstance()->getSema();
  ASTContext &Ctx = S.getASTContext();

  // Only include this header once and on demand. Because it's very heavy.
  static bool Included = false;
  if (!Included) {
    Included = true;
    llvm::cantFail(
        Interp.Parse("#include <__clang_interpreter_runtime_printvalue.h>"));
  }
  // Lookup std::string.
  NamespaceDecl *Std = LookupNamespace(S, "std");
  assert(Std && "Cannot find namespace std");
  Decl *StdStringDecl = LookupNamed(S, "string", Std);
  assert(StdStringDecl && "Cannot find std::string");
  const auto *StdStringTyDecl = llvm::dyn_cast<TypeDecl>(StdStringDecl);
  assert(StdStringTyDecl && "Cannot find type of std::string");

  // Create the wrapper function.
  DeclarationName DeclName =
      &Ctx.Idents.get(Interp.CreateUniqName(WrapperName));
  QualType RetTy(StdStringTyDecl->getTypeForDecl(), 0);
  QualType FnTy =
      Ctx.getFunctionType(RetTy, {}, FunctionProtoType::ExtProtoInfo());
  auto *WrapperFD = FunctionDecl::Create(
      Ctx, Ctx.getTranslationUnitDecl(), SourceLocation(), SourceLocation(),
      DeclName, FnTy, Ctx.getTrivialTypeSourceInfo(FnTy), SC_None);

  void *ValPtr = V.getPtr();
  if (!V.isManuallyAlloc())
    ValPtr = V.getPtrAddress();

  BuildWrapperBody(Interp, S, Ctx, WrapperFD, V.getType(), ValPtr);

  auto AddrOrErr = Interp.CompileDecl(WrapperFD);
  if (!AddrOrErr)
    llvm::logAllUnhandledErrors(AddrOrErr.takeError(), llvm::errs(),
                                "Fail to get symbol address");
  if (auto *Main = AddrOrErr->toPtr<std::string (*)()>())
    return (*Main)();
  return "Error to print the value!";
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const void *Ptr) {
  return PrintAddress(Ptr, '@');
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const void **Ptr) {
  return PrintAddress(*Ptr, '@');
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const bool *Val) {
  if (*Val)
    return "true";
  return "false";
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char *Val) {
  return PrintOneChar(*Val);
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const signed char *Val) {
  return PrintOneChar(*Val);
}

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned char *Val) {
  return PrintOneChar(*Val);
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const short *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned short *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const int *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned int *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long long *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned long *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string
PrintValueRuntime(const unsigned long long *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << *Val;
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const float *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << llvm::format("%#.6g", *Val) << 'f';
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const double *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << llvm::format("%#.12g", *Val);
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const long double *Val) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);
  SS << llvm::format("%#.8Lg", *Val) << 'L';
  return Str;
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char *const *Val) {
  return PrintString(Val);
}

REPL_EXTERNAL_VISIBILITY std::string PrintValueRuntime(const char **Val) {
  return PrintString(Val);
}

template <typename T> static std::string PrintValueWrapper(const T &Val) {
  return PrintValueRuntime(&Val);
}

namespace clang::caas {
std::string ReplPrintDataImpl(const Value &V) {
  std::string Str;
  llvm::raw_string_ostream SS(Str);

  QualType QT = V.getType();
  QualType DesugaredTy = QT.getDesugaredType(V.getASTContext());
  QualType NonRefTy = DesugaredTy.getNonReferenceType();

  if (NonRefTy->isNullPtrType())
    SS << "nullptr\n";
  else if (NonRefTy->isEnumeralType())
    return PrintEnum(V);
  else if (NonRefTy->isFunctionType())
    return PrintFunction(V, &V);
  else if ((NonRefTy->isPointerType() || NonRefTy->isMemberPointerType()) &&
           NonRefTy->getPointeeType()->isFunctionProtoType())
    return PrintFunction(V, V.getPtr());
  else if (auto *BT = DesugaredTy.getCanonicalType()->getAs<BuiltinType>()) {
    switch (BT->getKind()) {
    case BuiltinType::Bool: {
      SS << PrintValueWrapper(V.getBool());
      break;
    }
    case BuiltinType::Char_S:
    case BuiltinType::SChar: {
      SS << PrintValueWrapper(V.getSChar());
      break;
    }
    case BuiltinType::Short: {
      SS << PrintValueWrapper(V.getShort());
      break;
    }
    case BuiltinType::Int: {
      SS << PrintValueWrapper(V.getInt());
      break;
    }
    case BuiltinType::Long: {
      SS << PrintValueWrapper(V.getLong());
      break;
    }
    case BuiltinType::LongLong: {
      SS << PrintValueWrapper(V.getLongLong());
      break;
    }
    case BuiltinType::Char_U:
    case BuiltinType::UChar: {
      SS << PrintValueWrapper(V.getUChar());
      break;
    }
    case BuiltinType::UShort: {
      SS << PrintValueWrapper(V.getUShort());
      break;
    }
    case BuiltinType::UInt: {
      SS << PrintValueWrapper(V.getUInt());
      break;
    }
    case BuiltinType::ULong: {
      SS << PrintValueWrapper(V.getULong());
      break;
    }
    case BuiltinType::ULongLong: {
      SS << PrintValueWrapper(V.getULongLong());
      break;
    }
    case BuiltinType::Float: {
      SS << PrintValueWrapper(V.getFloat());
      break;
    }
    case BuiltinType::Double: {
      SS << PrintValueWrapper(V.getDouble());
      break;
    }
    case BuiltinType::LongDouble: {
      SS << PrintValueWrapper(V.getLongDouble());
      break;
    }
    default:
      llvm_unreachable("Unknown Builtintype kind");
    }
  } else if (auto *CXXRD = NonRefTy->getAsCXXRecordDecl();
             CXXRD && CXXRD->isLambda()) {
    SS << PrintAddress(V.getPtr(), '@');
  } else {
    // All fails then generate a runtime call, this is slow.
    SS << SynthesizeRuntimePrint(V);
  }
  return Str;
}

std::string ReplPrintTypeImpl(const Value &V) {
  ASTContext &Ctx = const_cast<ASTContext &>(V.getASTContext());
  QualType QT = V.getType();

  return PrintQualType(Ctx, QT);
}
} // namespace clang::caas
