#ifndef DEBUG_H_
#define DEBUG_H_
#define DEBUG 1
#if DEBUG
void debug_i(char *fmt,...);
void debug_w(char *fmt,...);
void debug_e(char *fmt,...);
void debug_d(char *fmt,...);
#else
static inline void debug_i(char *fmt,...)
{
}
static inline void debug_w(const char *fmt)
{
}
static inline void debug_e(const char *fmt)
{
}
static inline void debug_d(const char *fmt)
{
}
#endif

#endif /* DEBUG_H_ */
