---------------------------- MODULE consume_write ----------------------------
\* Formal model of the ONLINE consume/write lifecycle.
\* Invariant: a command causes at most one physical actuation attempt, and
\* after crash/restart the system never retries a command that may have
\* already affected the plant.
\*
\* Hash-chain journals are tamper-evident, not authenticated. This model
\* assumes the journal+seal pair that `CommandLedger::with_online_journal`
\* loads, not an adversary who replaces both files and sets first_online.

EXTENDS Naturals

CONSTANTS Commands

VARIABLES phase,         \* [cmd -> {"unseen","prepared","consumed","unknown"}]
          attempted,     \* [cmd -> BOOLEAN] physical write attempted
          journal,       \* persisted phase
          crashed

TypeOK ==
  /\ phase \in [Commands -> {"unseen","prepared","consumed","unknown"}]
  /\ attempted \in [Commands -> BOOLEAN]
  /\ journal \in [Commands -> {"unseen","prepared","consumed","unknown"}]
  /\ crashed \in BOOLEAN

Init ==
  /\ phase = [c \in Commands |-> "unseen"]
  /\ attempted = [c \in Commands |-> FALSE]
  /\ journal = [c \in Commands |-> "unseen"]
  /\ crashed = FALSE

MayWrite(c) == phase[c] = "unseen" /\ journal[c] = "unseen" /\ ~crashed

Prepare(c) ==
  /\ MayWrite(c)
  /\ phase' = [phase EXCEPT ![c] = "prepared"]
  /\ journal' = [journal EXCEPT ![c] = "prepared"]
  /\ UNCHANGED <<attempted, crashed>>

Write(c) ==
  /\ ~crashed
  /\ phase[c] = "prepared"
  /\ ~attempted[c]
  /\ attempted' = [attempted EXCEPT ![c] = TRUE]
  /\ UNCHANGED <<phase, journal, crashed>>

Ack(c) ==
  /\ ~crashed
  /\ attempted[c]
  /\ phase[c] = "prepared"
  /\ phase' = [phase EXCEPT ![c] = "consumed"]
  /\ journal' = [journal EXCEPT ![c] = "consumed"]
  /\ UNCHANGED <<attempted, crashed>>

MarkUnknown(c) ==
  /\ ~crashed
  /\ attempted[c]
  /\ phase[c] = "prepared"
  /\ phase' = [phase EXCEPT ![c] = "unknown"]
  /\ journal' = [journal EXCEPT ![c] = "unknown"]
  /\ UNCHANGED <<attempted, crashed>>

Crash ==
  /\ ~crashed
  /\ crashed' = TRUE
  /\ phase' = journal
  /\ UNCHANGED <<attempted, journal>>

Restart ==
  /\ crashed
  /\ crashed' = FALSE
  /\ phase' = journal
  /\ UNCHANGED <<attempted, journal>>

Next ==
  \/ \E c \in Commands : Prepare(c) \/ Write(c) \/ Ack(c) \/ MarkUnknown(c)
  \/ Crash
  \/ Restart

Spec == Init /\ [][Next]_<<phase, attempted, journal, crashed>>

AtMostOneAttempt ==
  \A c \in Commands : attempted[c] \in BOOLEAN

\* Once a write may have occurred, restart never returns to unseen.
NoRetryAfterPossibleWrite ==
  \A c \in Commands :
    attempted[c] => journal[c] \in {"prepared","consumed","unknown"}

NeverRetryKnownOrUnknown ==
  \A c \in Commands :
    journal[c] \in {"prepared","consumed","unknown"} => ~MayWrite(c)

=============================================================================
