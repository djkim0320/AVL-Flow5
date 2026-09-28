// Only the selected run may update the result panel, including late poll errors.
export function resultSelection({ read, render, pending, error }) {
  let selected = null,
    revision = 0;
  const capture = () => {
    const token = revision;
    return () => token === revision;
  };
  async function load(id, isCurrent) {
    try {
      const result = await read(id);
      if (isCurrent()) render(result);
    } catch (e) {
      if (isCurrent()) error(e);
    }
  }
  return {
    get id() {
      return selected;
    },
    capture,
    select(id, result) {
      selected = id;
      revision++;
      pending(id);
      if (result) {
        render(result);
        return Promise.resolve();
      }
      return load(id, capture());
    },
    refresh() {
      return selected ? load(selected, capture()) : Promise.resolve();
    }
  };
}
