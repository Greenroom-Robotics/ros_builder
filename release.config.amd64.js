
module.exports = {
  branches: ['main', '3.x'],
  plugins:
    [
      [
        '@semantic-release/commit-analyzer',
        { 'preset': 'conventionalcommits' }
      ],
      [
        '@semantic-release/release-notes-generator',
        { 'preset': 'conventionalcommits' }
      ],
      [
        "@semantic-release/exec",
        {
          "prepareCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch amd64 --ros_distro jazzy",
          "publishCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch amd64 --ros_distro jazzy --push"
        }
      ],
      [
        "@semantic-release/exec",
        {
          "publishCmd": 'echo "published=true" >> "$GITHUB_OUTPUT" && echo "version=${nextRelease.version}" >> "$GITHUB_OUTPUT"'
        }
      ],
      [
        "@semantic-release/github",
        {
          "successComment": false,
          "failComment": false
        }
      ]
    ],
};