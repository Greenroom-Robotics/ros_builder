
module.exports = {
  branches: ['main', 'add-integration-service-to-gama'],
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
          "prepareCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro humble",
          "publishCmd": "scripts/docker-build.py --version ${nextRelease.version} --arch arm64 --ros_distro humble --push true"
        }
      ]
    ],
};