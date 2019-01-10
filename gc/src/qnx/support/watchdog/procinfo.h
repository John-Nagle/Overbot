#ifndef PROCINFO_H
#define PROCINFO_H
int getprocinfo(const char* node, pid_t pid, pid_t& parent, pid_t& child, pid_t& sibling);
#endif // PROCINFO_H
