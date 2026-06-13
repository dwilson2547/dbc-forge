package com.robo.phonecompanion.data.git

import org.eclipse.jgit.api.Git
import org.eclipse.jgit.api.ResetCommand
import org.eclipse.jgit.merge.ContentMergeStrategy
import org.eclipse.jgit.merge.MergeStrategy
import org.eclipse.jgit.transport.UsernamePasswordCredentialsProvider
import java.io.File

/**
 * Wraps JGit for the operations needed to sync DBCs, sidecars, and vehicle
 * profiles to a remote HTTPS git repository authenticated via PAT.
 *
 * All methods are blocking — call from a background thread or coroutine.
 */
class GitRepository(private val workDir: File) {

    data class SyncStatus(
        val added: List<String>,
        val modified: List<String>,
        val deleted: List<String>,
    ) {
        val hasChanges: Boolean get() = added.isNotEmpty() || modified.isNotEmpty() || deleted.isNotEmpty()
    }

    private fun creds(token: String) =
        UsernamePasswordCredentialsProvider("token", token)

    val isInitialized: Boolean get() = File(workDir, ".git").exists()

    fun clone(repoUrl: String, token: String) {
        if (workDir.exists()) workDir.deleteRecursively()
        workDir.mkdirs()
        Git.cloneRepository()
            .setURI(repoUrl)
            .setDirectory(workDir)
            .setCredentialsProvider(creds(token))
            .call()
            .close()
    }

    fun pull(token: String) {
        openGit().use { git ->
            git.pull()
                .setCredentialsProvider(creds(token))
                .call()
        }
    }

    fun status(): SyncStatus {
        openGit().use { git ->
            val s = git.status().call()
            return SyncStatus(
                added = (s.untracked + s.added).sorted(),
                modified = (s.modified + s.changed).sorted(),
                deleted = (s.removed + s.missing).sorted(),
            )
        }
    }

    /**
     * Stage [paths] (relative to workDir), create a commit with [message],
     * then push. Returns the abbreviated commit hash.
     */
    fun commitAndPush(paths: List<String>, message: String, token: String): String {
        openGit().use { git ->
            val addCmd = git.add()
            paths.forEach { addCmd.addFilepattern(it) }
            addCmd.call()

            val commit = git.commit()
                .setMessage(message)
                .setAuthor("Phone Companion", "phone-companion@local")
                .call()

            git.push()
                .setCredentialsProvider(creds(token))
                .call()

            return commit.abbreviate(8).name()
        }
    }

    /**
     * Commit local changes, integrate the remote, and push — in one operation.
     *
     * Commit-first (rather than stash/pull/stash-apply): `stashCreate()` silently drops
     * **untracked** files, and new DBCs / vehicle profiles / sidecars created on the phone
     * are untracked — so the old flow stashed nothing, then `stashApply()` failed and was
     * mis-reported as a "merge conflict", and the push never ran. Committing first captures
     * untracked files correctly.
     *
     * Remote changes are merged in; on a *content* conflict the phone's edit wins
     * (`ContentMergeStrategy.OURS`) — the user just made it intentionally, and the remote
     * history still holds the other version. Only an unresolvable structural conflict
     * (e.g. edit-vs-delete) surfaces as an error, and even then the local commit is kept.
     */
    fun sync(paths: List<String>, message: String, token: String): String {
        openGit().use { git ->
            // 1. Stage new/modified files and deletions under the synced paths.
            git.add().apply { paths.forEach { addFilepattern(it) } }.call()
            git.add().setUpdate(true).apply { paths.forEach { addFilepattern(it) } }.call()

            // 2. Commit whatever is staged (no-op if the edits matched HEAD exactly).
            val staged = git.status().call().let {
                it.added.isNotEmpty() || it.changed.isNotEmpty() || it.removed.isNotEmpty()
            }
            if (staged) {
                git.commit()
                    .setMessage(message)
                    .setAuthor("Phone Companion", "phone-companion@local")
                    .setCommitter("Phone Companion", "phone-companion@local")
                    .call()
            }

            // 3. Fetch + merge the remote (favouring our content on conflict).
            git.fetch().setCredentialsProvider(creds(token)).call()
            val fetchHead = git.repository.resolve("FETCH_HEAD")
            if (fetchHead != null) {
                val merge = git.merge()
                    .include(fetchHead)
                    .setStrategy(MergeStrategy.RECURSIVE)
                    .setContentMergeStrategy(ContentMergeStrategy.OURS)
                    .setCommit(true)
                    .setMessage("Merge remote into phone edits")
                    .call()
                if (!merge.mergeStatus.isSuccessful) {
                    git.reset().setMode(ResetCommand.ResetType.HARD).setRef("HEAD").call()
                    throw GitSyncConflictException(
                        "Remote and phone changed the same file in a way that can't be " +
                            "auto-merged. Resolve it on the desktop and push, then Pull on the phone.",
                    )
                }
            }

            // 4. Push our commit (and any merge commit).
            git.push().setCredentialsProvider(creds(token)).call()

            return git.repository.resolve("HEAD").abbreviate(8).name()
        }
    }

    /**
     * Returns the log of commits from HEAD back [maxCount] entries.
     * Each entry is a pair of (abbreviated hash, subject line).
     */
    fun log(maxCount: Int = 10): List<Pair<String, String>> {
        openGit().use { git ->
            return git.log().setMaxCount(maxCount).call().map { commit ->
                commit.abbreviate(8).name() to commit.shortMessage
            }
        }
    }

    private fun openGit(): Git {
        check(isInitialized) { "No git repository at $workDir — call clone() first" }
        return Git.open(workDir)
    }
}

class GitSyncConflictException(message: String, cause: Throwable? = null) :
    Exception(message, cause)
