#ifndef GETOPT_H_
#define GETOPT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * get option letter from argument vector
 */
extern int opterr;   /* if error message should be printed */
extern int optind;   /* index into parent argv vector */
extern int optopt;   /* character checked for validity */
extern char *optarg; /* argument associated with option */

extern int getopt(int nargc, const char **nargv, const char *ostr);

#endif /* GETOPT_H_ */

#ifdef __cplusplus
}
#endif
