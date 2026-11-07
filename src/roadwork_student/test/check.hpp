// Minimal dependency-free test macros (return code drives ament_add_test).
#pragma once
#include <cstdio>

static int g_failures = 0, g_checks = 0;
#define CHECK(cond)                                                               \
  do {                                                                            \
    ++g_checks;                                                                   \
    if (!(cond)) {                                                                \
      ++g_failures;                                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                 \
    }                                                                             \
  } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::abs((a) - (b)) <= (tol))
#define TEST_SUMMARY(name)                                                        \
  (std::printf("%s: %d/%d checks passed\n", name, g_checks - g_failures, g_checks), g_failures == 0 ? 0 : 1)
