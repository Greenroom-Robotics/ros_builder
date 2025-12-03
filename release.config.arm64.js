
module.exports = {
  branches: ['3.x', 'main'],
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
          "prepareCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro kilted",
          "publishCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro kilted --push true"
        }
      ]
    ],
};
