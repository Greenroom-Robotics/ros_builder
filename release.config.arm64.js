
module.exports = {
  branches: ['3.x', 'main', 'feature/lyrical'],
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
          "prepareCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro lyrical",
          "publishCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro lyrical --push"
        }
      ]
    ],
};
