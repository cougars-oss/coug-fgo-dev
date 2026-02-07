### PR Checklist:
- [ ] A brief summary of the PR is provided.
- [ ] Appropriate tests and documentation are added.
- [ ] The changes build correctly: `colcon build`.
- [ ] The changes pass all tests and linting: `colcon test`.

--

- [ ] ROS package dependencies are added to the appropriate `package.xml`.
- [ ] System dependencies are added to the `Dockerfile`.
- [ ] External ROS packages are added to `cougars.repos`.
- [ ] Dependencies verified locally using `docker compose up --build -d`.

--

- [ ] Functionality verified in HoloOcean simulation.
- [ ] Functionality verified in the field.
