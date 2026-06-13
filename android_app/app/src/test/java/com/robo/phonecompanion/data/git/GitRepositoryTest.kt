package com.robo.phonecompanion.data.git

import org.eclipse.jgit.api.Git
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import java.io.File
import java.nio.file.Files

/**
 * End-to-end git sync against a local bare "remote". Reproduces the bug that broke phone
 * pushes: a new (untracked) DBC file. The old stash/pull/stash-apply flow dropped untracked
 * files and reported a false "merge conflict"; the commit-first flow must push it cleanly.
 */
class GitRepositoryTest {

    private lateinit var tmp: File
    private lateinit var remote: File
    private val token = "x"   // ignored by JGit for local file transport

    @Before
    fun setup() {
        tmp = Files.createTempDirectory("gitrepotest").toFile()
        remote = File(tmp, "remote.git")
        Git.init().setBare(true).setDirectory(remote).call().close()
        // Seed the remote with an initial commit so it has a branch to clone.
        val seed = File(tmp, "seed")
        Git.cloneRepository().setURI(remote.absolutePath).setDirectory(seed).call().use { g ->
            File(seed, "dbcs").mkdirs()
            File(seed, "dbcs/seed.txt").writeText("seed\n")
            g.add().addFilepattern("dbcs/seed.txt").call()
            g.commit().setMessage("seed").setAuthor("t", "t@t").setCommitter("t", "t@t").call()
            g.push().call()
        }
        seed.deleteRecursively()
    }

    @After
    fun teardown() {
        tmp.deleteRecursively()
    }

    private fun freshClone(name: String): File {
        val d = File(tmp, name)
        Git.cloneRepository().setURI(remote.absolutePath).setDirectory(d).call().close()
        return d
    }

    @Test
    fun `sync pushes a brand-new untracked file`() {
        val work = File(tmp, "phone")
        val repo = GitRepository(work)
        repo.clone(remote.absolutePath, token)

        File(work, "dbcs").mkdirs()
        File(work, "dbcs/new.dbc").writeText("BO_ 100 MSG: 8 ECU\n")  // untracked — the bug case

        val hash = repo.sync(listOf("dbcs/new.dbc"), "add new dbc", token)
        assertTrue("returned a commit hash", hash.isNotEmpty())

        val verify = freshClone("verify_new")
        assertTrue("remote received the new file", File(verify, "dbcs/new.dbc").exists())
    }

    @Test
    fun `sync merges a non-conflicting remote change and pushes`() {
        val work = File(tmp, "phone")
        val repo = GitRepository(work)
        repo.clone(remote.absolutePath, token)

        // Remote adds a different file in the meantime.
        val other = freshClone("other_nonconflict")
        Git.open(other).use { g ->
            File(other, "dbcs/remote.dbc").writeText("remote\n")
            g.add().addFilepattern("dbcs/remote.dbc").call()
            g.commit().setMessage("remote add").setAuthor("o", "o@o").setCommitter("o", "o@o").call()
            g.push().call()
        }

        // Phone adds its own file and syncs.
        File(work, "dbcs/phone.dbc").writeText("phone\n")
        repo.sync(listOf("dbcs/phone.dbc"), "phone add", token)

        val verify = freshClone("verify_merge")
        assertTrue(File(verify, "dbcs/phone.dbc").exists())   // ours pushed
        assertTrue(File(verify, "dbcs/remote.dbc").exists())  // theirs preserved
    }

    @Test
    fun `content conflict resolves in favour of the phone edit`() {
        val work = File(tmp, "phone")
        val repo = GitRepository(work)
        repo.clone(remote.absolutePath, token)

        // Remote changes seed.txt.
        val other = freshClone("other_conflict")
        Git.open(other).use { g ->
            File(other, "dbcs/seed.txt").writeText("remote-change\n")
            g.add().addFilepattern("dbcs/seed.txt").call()
            g.commit().setMessage("remote edit").setAuthor("o", "o@o").setCommitter("o", "o@o").call()
            g.push().call()
        }

        // Phone changes the same file differently, then syncs.
        File(work, "dbcs/seed.txt").writeText("phone-change\n")
        repo.sync(listOf("dbcs/seed.txt"), "phone edit", token)

        val verify = freshClone("verify_conflict")
        assertEquals("phone-change\n", File(verify, "dbcs/seed.txt").readText())
    }
}
