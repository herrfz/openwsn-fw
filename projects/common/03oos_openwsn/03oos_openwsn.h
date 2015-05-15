/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#ifndef __openwsn_H
#define __openwsn_H

int openwsn_start_thread(int argc, char **argv);

//=========================== define ==========================================

#define PRIORITY_OPENWSN            PRIORITY_MAIN-1

#endif
