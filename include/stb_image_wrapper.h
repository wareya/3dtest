#if _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
    #endif
    #include <windows.h>
    #include <string>
    static inline FILE* wrap_fopen(const char * X, const char * Y)
    {
        size_t Y_S = MultiByteToWideChar(CP_UTF8, 0, Y, -1, NULL, 0)*sizeof(wchar_t);
        size_t X_S = MultiByteToWideChar(CP_UTF8, 0, X, -1, NULL, 0)*sizeof(wchar_t);
        std::wstring W_Y(Y_S, L'#');
        std::wstring W_X(X_S, L'#');
        MultiByteToWideChar(CP_UTF8, 0, Y, -1, &W_Y[0], Y_S);
        MultiByteToWideChar(CP_UTF8, 0, X, -1, &W_X[0], X_S);
        return _wfopen(W_X.data(),W_Y.data());
    }
    #define fopen wrap_fopen
#endif
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#if _WIN32
    #undef fopen
#endif
