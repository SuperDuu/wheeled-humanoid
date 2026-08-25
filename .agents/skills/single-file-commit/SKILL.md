---
name: single-file-commit
description: Enforce strict single-file git commits with concise 2-3 word English commit messages for every code modification.
---

# Single-File Git Commit Rule

Whenever making modifications to the workspace or committing code changes:

1. **One Commit Per File**:
   - Each modified or added file MUST be committed in its own individual git commit (`git add <single_file> && git commit -m "..."`).
   - NEVER bundle or combine multiple files into a single commit.

2. **Concise Commit Message**:
   - The commit message MUST be strictly 2 to 3 words in English.
   - Format: `<verb> <noun>` or `<verb> <adjective> <noun>`.
   - Examples:
     - `update dynamic align`
     - `tune speed pi`
     - `add anti stiction`
     - `update vesc conf`
     - `clean foc math`
